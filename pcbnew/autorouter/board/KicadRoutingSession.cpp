/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Host-side proposal validation and repair. The routing algorithm remains data-only.
 */
#include "KicadRoutingSession.h"
#include "../AutorouterDebug.h"
#include "KicadBoardAdapter.h"
#include "../pipeline/RoutingPipeline.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ratsnest/ratsnest_data.h>
#include <map>
#include <set>
#include <stdexcept>
#include <board.h>
#include <board_design_settings.h>
#include <connectivity/connectivity_data.h>
#include <drc/drc_engine.h>
#include <drc/drc_item.h>
#include <pcb_io/kicad_sexpr/pcb_io_kicad_sexpr.h>
#include <pcb_track.h>
#include <project/net_settings.h>
#include <progress_reporter.h>
#include <richio.h>
#include <zone_filler.h>

namespace KICAD_AUTOROUTER
{
namespace
{
using CLOCK = std::chrono::steady_clock;
constexpr int AUTOROUTER_DRC_ERROR_LIMIT = 10000;

class CANCEL_REPORTER : public PROGRESS_REPORTER
{
public:
    explicit CANCEL_REPORTER( ROUTER_CANCEL_CALLBACK aCancel ) : cancel( std::move( aCancel ) ) {}
    bool IsCancelled() const override { return cancel && cancel(); }
    bool KeepRefreshing( bool = false ) override { return !IsCancelled(); }
    void SetNumPhases( int ) override {}
    void AddPhases( int ) override {}
    void BeginPhase( int ) override {}
    void AdvancePhase() override {}
    void AdvancePhase( const wxString& ) override {}
    void Report( const wxString& ) override {}
    void SetCurrentProgress( double ) override {}
    void SetMaxProgress( int ) override {}
    void AdvanceProgress() override {}
    void SetTitle( const wxString& ) override {}
private:
    ROUTER_CANCEL_CALLBACK cancel;
};

std::unique_ptr<BOARD> copyBoard( BOARD& source )
{
    // Native in-memory serialization preserves UUIDs, custom pad shapes,
    // groups, zones and unsaved edits without maintaining a second clone schema.
    PCB_IO_KICAD_SEXPR io;
    STRING_FORMATTER text;
    io.FormatBoardToFormatter( &text, &source, nullptr, false );
    std::unique_ptr<BOARD> result( dynamic_cast<BOARD*>( io.Parse(
            wxString::FromUTF8( text.GetString() ) ) ) );
    if( !result )
        throw std::runtime_error( "Could not create private routing board" );
    result->SetFileName( source.GetFileName() );
    result->SetDesignSettings( source.GetDesignSettings() );
    auto netSettings = std::make_shared<NET_SETTINGS>( nullptr, "" );
    netSettings->CopyFrom( *source.GetDesignSettings().m_NetSettings );
    result->GetDesignSettings().m_NetSettings = std::move( netSettings );
    // Never share the source engine: its board, caches and handlers are mutable.
    result->GetDesignSettings().m_DRCEngine.reset();
    result->SynchronizeNetsAndNetClasses( false );
    result->BuildConnectivity();
    return result;
}

void initDrc( BOARD& board, CANCEL_REPORTER& reporter )
{
    auto engine = std::make_shared<DRC_ENGINE>( &board, &board.GetDesignSettings() );
    wxFileName rules( board.GetFileName() );
    rules.SetExt( "kicad_dru" );
    engine->InitEngine( rules.Exists() ? rules : wxFileName() );
    // The standard DRC output cap is deliberately small for an interactive
    // report.  A private autorouter session instead needs the complete
    // pre-route baseline so existing board errors cannot make validation fail
    // before it has routed a single task.  Keep the unconnected-item cap: the
    // host connectivity graph supplies that count independently.
    for( int code = DRCE_FIRST; code <= DRCE_LAST; ++code )
    {
        if( code != DRCE_UNCONNECTED_ITEMS )
            engine->SetErrorLimitOverride( code, AUTOROUTER_DRC_ERROR_LIMIT );
    }
    engine->SetProgressReporter( &reporter );
    board.GetDesignSettings().m_DRCEngine = std::move( engine );
}

void refill( BOARD& board, CANCEL_REPORTER& reporter )
{
    ZONE_FILLER filler( &board, nullptr );
    filler.SetProgressReporter( &reporter );
    if( !board.Zones().empty() && !filler.Fill( board.Zones() ) )
        throw std::runtime_error( reporter.IsCancelled() ? "Autorouter cancelled"
                                                        : "Proposal zone refill failed" );
    board.BuildConnectivity();
    board.GetConnectivity()->RecalculateRatsnest();
}

using VIOLATIONS = std::map<std::string, int>;
VIOLATIONS violations( BOARD& board )
{
    VIOLATIONS result;
    auto engine = board.GetDesignSettings().m_DRCEngine;
    engine->SetViolationHandler( [&]( const std::shared_ptr<DRC_ITEM>& item,
                                      const VECTOR2I& position, int layer,
                                      const std::function<void( PCB_MARKER* )>& )
    {
        if( !item || item->GetErrorCode() == DRCE_UNCONNECTED_ITEMS
            || board.GetDesignSettings().GetSeverity( item->GetErrorCode() )
                    == SEVERITY::RPT_SEVERITY_IGNORE )
            return;
        autorouterDebugLog( "KiCad DRC " + std::to_string( item->GetErrorCode() ) + " at "
                + std::to_string( position.x ) + "," + std::to_string( position.y ) + ": "
                + item->GetErrorMessage( false ).ToStdString() );
        std::vector<std::string> ids{ item->GetMainItemID().AsString().ToStdString(),
                item->GetAuxItemID().AsString().ToStdString(),
                item->GetAuxItem2ID().AsString().ToStdString(),
                item->GetAuxItem3ID().AsString().ToStdString() };
        std::sort( ids.begin(), ids.end() );
        std::string key = std::to_string( item->GetErrorCode() ) + "@"
                + std::to_string( position.x ) + "," + std::to_string( position.y )
                + "/" + std::to_string( layer );
        for( const auto& id : ids )
            key += "/" + id;
        ++result[key];
    } );
    try { engine->RunTests( EDA_UNITS::MM, true, false ); }
    catch( ... ) { engine->ClearViolationHandler(); throw; }
    engine->ClearViolationHandler();
    if( !engine->TestsCompleted() )
        throw std::runtime_error( "KiCad design-rule checking did not complete" );
    for( int code = DRCE_FIRST; code <= DRCE_LAST; ++code )
        // Connectivity is counted independently without the marker limit.
        // A large initially unrouted board is not a failed clearance check.
        if( code != DRCE_UNCONNECTED_ITEMS
            && board.GetDesignSettings().GetSeverity( code ) != SEVERITY::RPT_SEVERITY_IGNORE
            && engine->IsErrorLimitExceeded( code ) )
            throw std::runtime_error( "KiCad DRC reached its error limit; proposal is not validated" );
    return result;
}

int introduced( const VIOLATIONS& current, const VIOLATIONS& baseline )
{
    int count = 0;
    for( const auto& [key, value] : current )
    {
        const auto found = baseline.find( key );
        count += std::max( 0, value - ( found == baseline.end() ? 0 : found->second ) );
    }
    return count;
}

void apply( BOARD& board, const ROUTING_RESULT& result )
{
    KICAD_BOARD_ADAPTER adapter( &board );
    for( const auto& id : result.removedBoardItemIds )
        if( auto* item = adapter.FindBoardItem( id ) )
        { board.Remove( item ); delete item; }
    auto items = adapter.CreatePreviewItems( result );
    for( auto& item : items )
    {
        item->ClearFlags( ROUTER_TRANSIENT );
        board.Add( item.release() );
    }
}

std::set<std::string> autorouterOwnedTracks(
        const BOARD& board, const std::set<std::string>& sourceTracks )
{
    std::set<std::string> result;

    for( const PCB_TRACK* track : board.Tracks() )
    {
        if( !track )
            continue;

        const std::string id = track->m_Uuid.AsString().ToStdString();
        if( !sourceTracks.contains( id ) )
            result.insert( id );
    }

    return result;
}
}

struct KICAD_ROUTING_SESSION::IMPL
{
    std::unique_ptr<BOARD> board;
    std::map<wxString, int> sourceNets;
    std::set<std::string> sourceTracks;
    bool used = false;
};

KICAD_ROUTING_SESSION::KICAD_ROUTING_SESSION( BOARD& source ) : m_impl( std::make_unique<IMPL>() )
{
    m_impl->board = copyBoard( source );
    for( auto* net : source.GetNetInfo() )
        m_impl->sourceNets[net->GetNetname()] = net->GetNetCode();
    for( auto* track : source.Tracks() )
        m_impl->sourceTracks.insert( track->m_Uuid.AsString().ToStdString() );
}
KICAD_ROUTING_SESSION::~KICAD_ROUTING_SESSION() = default;

ROUTING_RESULT KICAD_ROUTING_SESSION::Run( const AUTOROUTER_SETTINGS& settings,
        const ROUTER_CANCEL_CALLBACK& cancel, const ROUTER_PROGRESS_CALLBACK& progress )
{
    if( m_impl->used )
        throw std::logic_error( "A routing session can only be run once" );
    m_impl->used = true;
    const auto started = CLOCK::now();
    CANCEL_REPORTER reporter( cancel );
    auto& work = m_impl->board;
    initDrc( *work, reporter );
    struct DETACH_REPORTER
    {
        std::unique_ptr<BOARD>& board;
        ~DETACH_REPORTER()
        {
            if( board && board->GetDesignSettings().m_DRCEngine )
                board->GetDesignSettings().m_DRCEngine->SetProgressReporter( nullptr );
        }
    } detach{ work };
    ROUTER_PROGRESS lastProgress;
    const auto recordProgress = [&]( const ROUTER_PROGRESS& state )
    {
        lastProgress = state;
        if( progress )
            progress( state );
    };
    auto stage = [&]( const char* message )
    {
        if( progress )
        {
            ROUTER_PROGRESS state = lastProgress;
            state.stage = message;
            state.elapsedMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
                    CLOCK::now() - started ).count();
            progress( state );
        }
        if( reporter.IsCancelled() )
            throw std::runtime_error( "Autorouter cancelled" );
    };
    stage( "Checking input copper and design rules" );
    refill( *work, reporter );
    const auto baseline = violations( *work );
    auto snapshot = KICAD_BOARD_ADAPTER( work.get() ).CreateSnapshot( settings );
    if( !snapshot )
        throw std::runtime_error( "Could not create an autorouter snapshot from the board" );

    std::size_t snapshotConnections = 0;

    for( const ROUTING_NET& net : snapshot->nets )
        snapshotConnections += net.connections.size();

    autorouterDebugLog( "Host snapshot: pads=" + std::to_string( snapshot->pads.size() )
                        + " nets=" + std::to_string( snapshot->nets.size() )
                        + " connections=" + std::to_string( snapshotConnections )
                        + " obstacles=" + std::to_string( snapshot->obstacles.size() ) );
    if( autorouterDebugEnabled() )
    {
        for( const ROUTING_NET& net : snapshot->nets )
        {
            autorouterDebugLog( "Host snapshot net code=" + std::to_string( net.netCode )
                                + " name=" + net.name );
        }
    }
    const auto routingStarted = CLOCK::now();
    auto result = ROUTING_PIPELINE().Run( *snapshot, settings, cancel, recordProgress );
    std::int64_t routingMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            CLOCK::now() - routingStarted ).count();
    if( result.cancelled || reporter.IsCancelled() )
    { result.cancelled = true; result.complete = false; return result; }
    autorouterDebugLog( "Pipeline result: routed=" + std::to_string( result.metrics.routedConnections )
                        + "/" + std::to_string( result.metrics.totalConnections )
                        + " segments=" + std::to_string( result.segments.size() )
                        + " vias=" + std::to_string( result.vias.size() )
                        + " complete=" + std::to_string( result.complete )
                        + " message=" + result.message );
    apply( *work, result );
    std::set<std::string> jobOwnedTracks =
            autorouterOwnedTracks( *work, m_impl->sourceTracks );
    stage( "Refilling and validating the proposal" );
    refill( *work, reporter );
    int missing = work->GetConnectivity()->GetUnconnectedCount( false );
    autorouterDebugLog( "Host missing after first refill=" + std::to_string( missing ) );
    int newDrc = introduced( violations( *work ), baseline );
    // Re-evaluate every host ratsnest, including nets with zero initial tasks.
    // Strict improvement and a bounded pass count prevent refill/retry loops.
    for( int pass = 0; missing > 0 && pass < std::max( 1, settings.maxPasses ); ++pass )
    {
        stage( "Repairing connections found after zone refill" );
        auto candidate = copyBoard( *work );
        initDrc( *candidate, reporter );
        AUTOROUTER_SETTINGS repairSettings = settings;
        repairSettings.allowRipupExisting = false;
        // With no legal via escape, a strict raster can spend the entire
        // node budget proving that a crossing-free repair is impossible.
        // Multilayer repair retains the strict first attempt for now: the
        // native batch scheduler can otherwise oscillate two short routes in
        // a narrow channel instead of selecting the reference shove result.
        repairSettings.allowRipupOnFirstIteration = !repairSettings.allowVias;
        repairSettings.routeOnlyUnconnected = true;
        repairSettings.enableFanout = false;
        auto repairSnapshot = KICAD_BOARD_ADAPTER(
                candidate.get(), jobOwnedTracks ).CreateSnapshot( repairSettings );
        const auto repairStarted = CLOCK::now();
        auto repair = ROUTING_PIPELINE().Run( *repairSnapshot, repairSettings, cancel, recordProgress );
        routingMs += std::chrono::duration_cast<std::chrono::milliseconds>(
                CLOCK::now() - repairStarted ).count();
        autorouterDebugLog( "Repair pipeline result: routed="
                + std::to_string( repair.metrics.routedConnections ) + "/"
                + std::to_string( repair.metrics.totalConnections ) + " segments="
                + std::to_string( repair.segments.size() ) + " vias="
                + std::to_string( repair.vias.size() ) + " removed="
                + std::to_string( repair.removedBoardItemIds.size() ) + " ripups="
                + std::to_string( repair.metrics.ripups ) );
        if( autorouterDebugEnabled() )
        {
            KICAD_BOARD_ADAPTER candidateAdapter( candidate.get() );
            for( const std::string& id : repair.removedBoardItemIds )
            {
                const BOARD_ITEM* item = candidateAdapter.FindBoardItem( id );
                const PCB_TRACK* track = dynamic_cast<const PCB_TRACK*>( item );
                autorouterDebugLog( "Repair removes id=" + id
                        + " net=" + std::to_string( track ? track->GetNetCode() : -1 ) );
            }
        }
        if( repair.cancelled || reporter.IsCancelled() )
        { result.cancelled = true; result.complete = false; return result; }
        if( repair.segments.empty() && repair.vias.empty() )
            break;
        apply( *candidate, repair );
        stage( "Checking repaired copper and design rules" );
        refill( *candidate, reporter );
        const int candidateMissing = candidate->GetConnectivity()->GetUnconnectedCount( false );
        const int candidateDrc = introduced( violations( *candidate ), baseline );
        autorouterDebugLog( "Host repair missing=" + std::to_string( candidateMissing )
                + " newDrc=" + std::to_string( candidateDrc ) );
        if( candidateMissing >= missing || candidateDrc > newDrc )
            break;
        work = std::move( candidate );
        jobOwnedTracks = autorouterOwnedTracks( *work, m_impl->sourceTracks );
        missing = candidateMissing;
        newDrc = candidateDrc;
        ++result.hostRepairPasses;
        result.metrics.expandedNodes += repair.metrics.expandedNodes;
        result.metrics.ripups += repair.metrics.ripups;
    }
    // Export the actual retained candidate, not stale request vectors. UUIDs
    // distinguish pre-existing copper; net names remap serialization's codes.
    result.segments.clear();
    result.vias.clear();
    result.connections.clear();
    result.removedBoardItemIds.clear();
    std::set<std::string> retained;
    for( auto* track : work->Tracks() )
    {
        const auto id = track->m_Uuid.AsString().ToStdString();
        if( m_impl->sourceTracks.contains( id ) )
        { retained.insert( id ); continue; }
        const int net = m_impl->sourceNets.at( track->GetNetname() );
        const auto p = track->GetStart();
        if( auto* via = dynamic_cast<PCB_VIA*>( track ) )
        {
            ROUTING_VIA output;
            output.netCode = net;
            output.position = { p.x, p.y };
            output.topLayer = via->TopLayer();
            output.bottomLayer = via->BottomLayer();
            output.diameter = via->GetWidth();
            output.drill = via->GetDrill();
            for( auto layer : via->GetLayerSet().Seq() )
                output.layers.push_back( layer );
            result.vias.push_back( output );
        }
        else
        {
            const auto q = track->GetEnd();
            result.segments.push_back( { net, track->GetLayer(), { p.x, p.y }, { q.x, q.y }, track->GetWidth() } );
        }
    }
    for( const auto& id : m_impl->sourceTracks )
        if( !retained.contains( id ) )
            result.removedBoardItemIds.push_back( id );
    result.metrics.segmentCount = result.segments.size();
    result.metrics.viaCount = result.vias.size();
    result.metrics.routedLengthIU = 0;
    for( const auto& segment : result.segments )
        result.metrics.routedLengthIU += std::hypot( static_cast<double>( segment.end.x - segment.start.x ),
                                                   static_cast<double>( segment.end.y - segment.start.y ) );
    result.unroutedNetCodes.clear();
    for( auto* net : work->GetNetInfo() )
        if( auto* rn = work->GetConnectivity()->GetRatsnestForNet( net->GetNetCode() );
            rn && !rn->GetEdges().empty() )
            result.unroutedNetCodes.push_back( m_impl->sourceNets.at( net->GetNetname() ) );
    result.hostValidated = !reporter.IsCancelled();
    result.hostUnconnected = missing;
    result.hostNewDrcViolations = newDrc;
    result.complete = result.hostValidated && missing == 0 && newDrc == 0;
    result.metrics.drcViolations = newDrc;
    result.metrics.elapsedMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
            CLOCK::now() - started ).count();
    result.hostValidationMilliseconds = result.metrics.elapsedMilliseconds - routingMs;
    result.message = result.complete ? "Proposal passed KiCad connectivity and design-rule checks"
            : "Proposal is incomplete or introduces KiCad design-rule violations";
    work->GetDesignSettings().m_DRCEngine->SetProgressReporter( nullptr );
    return result;
}
}
