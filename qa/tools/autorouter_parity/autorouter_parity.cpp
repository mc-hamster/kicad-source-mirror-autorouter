/*
 * This program source code file is part of KiCad, a free EDA application.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 */

/*
 * Small, headless corpus harness for the native Freerouting-derived router.
 *
 * Native full-board runs use the editor's KICAD_ROUTING_SESSION, including
 * private refill, validation and repair. An additional, independent live DRC
 * pass applies the proposal to the original freshly loaded board. Bounded
 * raw-worker diagnosis and reference DSN/SES I/O are QA-only options.
 * The input file is never written.
 */

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <wx/init.h>
#include <wx/filename.h>

#include <board.h>
#include <board_design_settings.h>
#include <connectivity/connectivity_data.h>
#include <pcb_track.h>
#include <commit.h>
#include <zone_filler.h>
#include <drc/drc_engine.h>
#include <drc/drc_item.h>
#include <pgm_base.h>
#include <properties/property_mgr.h>
#include <pcbnew_utils/board_file_utils.h>
#include <specctra_import_export/specctra.h>

#include <autorouter/AutorouterTypes.h>
#include <autorouter/board/KicadBoardAdapter.h>
#include <autorouter/board/KicadRoutingSession.h>
#include <autorouter/pipeline/RoutingPipeline.h>


namespace
{

struct PARITY_PGM : public PGM_BASE
{
    void MacOpenFile( const wxString& ) override {}
};


PARITY_PGM g_program;


// The reference importer stages ordinary board edits. This commit applies
// them to a disposable QA board; it has no editor, view, or undo dependency.
class REFERENCE_IMPORT_COMMIT : public COMMIT
{
public:
    explicit REFERENCE_IMPORT_COMMIT( BOARD& aBoard ) : m_board( aBoard ) {}
    ~REFERENCE_IMPORT_COMMIT() override { Revert(); }

    void Push( const wxString&, int ) override
    {
        for( const COMMIT_LINE& line : m_entries )
        {
            BOARD_ITEM* item = static_cast<BOARD_ITEM*>( line.m_item );
            switch( line.m_type & CHT_TYPE )
            {
            case CHT_ADD: m_board.Add( item ); break;
            case CHT_REMOVE: m_board.Remove( item ); delete item; break;
            default: break; // Modifications were applied after Stage().
            }
            delete line.m_copy;
        }
        clear();
    }

    void Revert() override
    {
        for( const COMMIT_LINE& line : m_entries )
        {
            if( ( line.m_type & CHT_TYPE ) == CHT_ADD )
                delete line.m_item;
            else if( ( line.m_type & CHT_TYPE ) == CHT_MODIFY && line.m_copy )
                static_cast<BOARD_ITEM*>( line.m_item )->SwapItemData(
                        static_cast<BOARD_ITEM*>( line.m_copy ) );
            delete line.m_copy;
        }
        clear();
    }

    EDA_ITEM* ResolveItem( KIID& aID ) override
    {
        if( aID == niluuid )
            return nullptr;

        for( COMMIT_LINE& entry : m_entries )
        {
            if( entry.m_item && entry.m_item->IsBOARD_ITEM() && entry.m_item->m_Uuid == aID )
                return entry.m_item;
        }

        return nullptr;
    }

private:
    EDA_ITEM* undoLevelItem( EDA_ITEM* aItem ) const override { return aItem; }
    EDA_ITEM* makeImage( EDA_ITEM* aItem ) const override { return aItem->Clone(); }
    BOARD& m_board;
};


struct OPTIONS
{
    std::string board;
    std::string output;
    std::string exportDsn;
    std::string importSes;
    std::string saveBoard;
    bool        stripTracks = false;
    std::vector<std::string> includeNets;
    bool        dumpSnapshot = false;
    int         maxPasses = 4;
    int         maxIterations = 8;
    int         optimizationPasses = 2;
    int         maxExpandedNodes = 250000;
    int         maxNets = 0;
    int         maxConnections = 0;
    bool        fanout = true;
    bool        vias = true;
    bool        routeOnlyUnconnected = true;
    bool        liveDrc = true;
};


struct LIVE_DRC_RESULT
{
    int            count = -1;
    std::map<int, int> byCode;
    std::vector<std::string> details;
    std::map<std::string, int> fingerprints;
};


[[noreturn]] void usageError( const std::string& aMessage )
{
    throw std::invalid_argument( aMessage
                                 + "\nusage: qa_autorouter_parity --board FILE"
                                   " [--out FILE] [--max-passes N]"
                                   " [--max-iterations N] [--max-expanded-nodes N]"
                                   " [--optimization-passes N]"
                                   " [--no-fanout] [--no-vias] [--all-connections] [--no-live-drc]"
                                   " [--max-nets N] [--max-connections N]"
                                   " [--export-dsn FILE] [--include-net NET]"
                                   " [--strip-tracks] [--import-ses FILE]"
                                   " [--save-board NEW_FILE]"
                                   " [--dump-snapshot]" );
}


int parsePositive( const char* aName, const std::string& aValue )
{
    char* end = nullptr;
    const long value = std::strtol( aValue.c_str(), &end, 10 );

    if( end == aValue.c_str() || *end != '\0' || value < 1 || value > 1000000000L )
        usageError( std::string( aName ) + " requires a positive integer" );

    return static_cast<int>( value );
}


int parseNonNegative( const char* aName, const std::string& aValue )
{
    char* end = nullptr;
    const long value = std::strtol( aValue.c_str(), &end, 10 );

    if( end == aValue.c_str() || *end != '\0' || value < 0 || value > 1000000000L )
        usageError( std::string( aName ) + " requires a non-negative integer" );

    return static_cast<int>( value );
}


OPTIONS parseOptions( int argc, char** argv )
{
    OPTIONS options;

    for( int index = 1; index < argc; ++index )
    {
        const std::string_view argument( argv[index] );

        auto nextValue = [&]( const char* aName ) -> std::string
        {
            if( index + 1 >= argc )
                usageError( std::string( aName ) + " requires a value" );

            return argv[++index];
        };

        if( argument == "--board" )
            options.board = nextValue( "--board" );
        else if( argument == "--out" )
            options.output = nextValue( "--out" );
        else if( argument == "--export-dsn" )
            options.exportDsn = nextValue( "--export-dsn" );
        else if( argument == "--import-ses" )
            options.importSes = nextValue( "--import-ses" );
        else if( argument == "--save-board" )
            options.saveBoard = nextValue( "--save-board" );
        else if( argument == "--strip-tracks" )
            options.stripTracks = true;
        else if( argument == "--include-net" )
            options.includeNets.push_back( nextValue( "--include-net" ) );
        else if( argument == "--dump-snapshot" )
            options.dumpSnapshot = true;
        else if( argument == "--max-passes" )
            options.maxPasses = parsePositive( "--max-passes", nextValue( "--max-passes" ) );
        else if( argument == "--max-iterations" )
            options.maxIterations =
                    parsePositive( "--max-iterations", nextValue( "--max-iterations" ) );
        else if( argument == "--optimization-passes" )
            options.optimizationPasses =
                    parseNonNegative( "--optimization-passes", nextValue( "--optimization-passes" ) );
        else if( argument == "--max-expanded-nodes" )
            options.maxExpandedNodes =
                    parsePositive( "--max-expanded-nodes", nextValue( "--max-expanded-nodes" ) );
        else if( argument == "--max-nets" )
            options.maxNets = parsePositive( "--max-nets", nextValue( "--max-nets" ) );
        else if( argument == "--max-connections" )
            options.maxConnections =
                    parsePositive( "--max-connections", nextValue( "--max-connections" ) );
        else if( argument == "--no-fanout" )
            options.fanout = false;
        else if( argument == "--no-vias" )
        {
            options.vias = false;
            options.fanout = false;
        }
        else if( argument == "--all-connections" )
            options.routeOnlyUnconnected = false;
        else if( argument == "--no-live-drc" )
            options.liveDrc = false;
        else
            usageError( "unknown argument: " + std::string( argument ) );
    }

    if( options.board.empty() )
        usageError( "--board is required" );

    return options;
}


std::string jsonEscape( const std::string& aValue )
{
    std::ostringstream result;

    for( const unsigned char character : aValue )
    {
        switch( character )
        {
        case '\\': result << "\\\\"; break;
        case '"': result << "\\\""; break;
        case '\n': result << "\\n"; break;
        case '\r': result << "\\r"; break;
        case '\t': result << "\\t"; break;
        default:
            if( character < 0x20 )
            {
                result << "\\u00" << std::hex << static_cast<int>( character ) << std::dec;
            }
            else
            {
                result << character;
            }
            break;
        }
    }

    return result.str();
}


LIVE_DRC_RESULT runLiveDrc( BOARD* aBoard )
{
    if( !aBoard || !aBoard->GetDesignSettings().m_DRCEngine )
        return {};

    LIVE_DRC_RESULT result;
    result.count = 0;
    DRC_ENGINE* engine = aBoard->GetDesignSettings().m_DRCEngine.get();
    engine->SetViolationHandler(
            [&]( const std::shared_ptr<DRC_ITEM>& aItem, const VECTOR2I& aPosition, int aLayer,
                 const std::function<void( PCB_MARKER* )>& )
            {
                if( aItem && aBoard->GetDesignSettings().GetSeverity( aItem->GetErrorCode() )
                                     != SEVERITY::RPT_SEVERITY_IGNORE )
                {
                    ++result.count;
                    ++result.byCode[aItem->GetErrorCode()];
                    // Compare actual violations, not just totals per error code:
                    // repairing one old violation must not hide a new one.
                    // Unconnected items are measured independently by connectivity.
                    if( aItem->GetErrorCode() != DRCE_UNCONNECTED_ITEMS )
                    {
                        std::string key = wxString::Format( "%d@%d,%d/L%d:",
                                aItem->GetErrorCode(), aPosition.x, aPosition.y, aLayer )
                                                  .ToStdString();
                        std::vector<std::string> ids = {
                            aItem->GetMainItemID().AsString().ToStdString(),
                            aItem->GetAuxItemID().AsString().ToStdString(),
                            aItem->GetAuxItem2ID().AsString().ToStdString(),
                            aItem->GetAuxItem3ID().AsString().ToStdString() };
                        std::sort( ids.begin(), ids.end() );
                        for( const std::string& id : ids )
                            key += id + "/";
                        ++result.fingerprints[key];
                    }
                    result.details.push_back(
                            wxString::Format( "%d@(%d,%d)/L%d: %s", aItem->GetErrorCode(),
                                              aPosition.x, aPosition.y, aLayer,
                                              aItem->GetErrorMessage( false ) )
                                    .ToStdString() + " ["
                            + aItem->GetMainItemID().AsString().ToStdString() + ","
                            + aItem->GetAuxItemID().AsString().ToStdString() + "]" );
                }
            } );
    try { engine->RunTests( EDA_UNITS::MM, true, false ); }
    catch( ... ) { engine->ClearViolationHandler(); throw; }
    engine->ClearViolationHandler();
    if( !engine->TestsCompleted() )
        throw std::runtime_error( "independent KiCad DRC did not complete" );
    for( int code = DRCE_FIRST; code <= DRCE_LAST; ++code )
        if( code != DRCE_UNCONNECTED_ITEMS // Full connectivity is counted separately.
            && aBoard->GetDesignSettings().GetSeverity( code ) != SEVERITY::RPT_SEVERITY_IGNORE
            && engine->IsErrorLimitExceeded( code ) )
            throw std::runtime_error( "independent KiCad DRC reached its error limit" );
    return result;
}


void applyProposal( BOARD* aBoard,
                              const KICAD_AUTOROUTER::ROUTING_RESULT& aResult,
                              const KICAD_AUTOROUTER::KICAD_BOARD_ADAPTER& aAdapter )
{
    if( !aBoard )
        return;

    // Apply the same removals as acceptance in the editor.
    std::vector<PCB_TRACK*> removed;
    for( PCB_TRACK* track : aBoard->Tracks() )
    {
        if( std::find( aResult.removedBoardItemIds.begin(), aResult.removedBoardItemIds.end(),
                       track->m_Uuid.AsString().ToStdString() )
            != aResult.removedBoardItemIds.end() )
            removed.push_back( track );
    }
    for( PCB_TRACK* track : removed )
    {
        aBoard->Remove( track );
        delete track;
    }

    std::vector<std::unique_ptr<BOARD_ITEM>> preview = aAdapter.CreatePreviewItems( aResult );
    for( std::unique_ptr<BOARD_ITEM>& item : preview )
    {
        if( !item )
            continue;

        item->ClearFlags( ROUTER_TRANSIENT );
        aBoard->Add( item.release() );
    }

    // Connectivity-dependent DRC providers need the proposal in the board
    // connectivity index, just as it is after BOARD_COMMIT::Push in pcbnew.
    if( !aBoard->Zones().empty() && !ZONE_FILLER( aBoard, nullptr ).Fill( aBoard->Zones() ) )
        throw std::runtime_error( "proposal zone refill failed" );
    aBoard->BuildConnectivity();
    aBoard->GetConnectivity()->RecalculateRatsnest();
}


void writeJson( const std::string& aPath, const std::string& aBoardPath,
                const KICAD_AUTOROUTER::ROUTING_RESULT& aResult,
                const LIVE_DRC_RESULT& aBaselineDrc, const LIVE_DRC_RESULT& aLiveDrc,
                std::int64_t aRuntimeMs, std::int64_t aHostSessionMs,
                int aBaselineUnconnected, int aUnconnected )
{
    const KICAD_AUTOROUTER::ROUTER_METRICS& metrics = aResult.metrics;
    int introducedDrcErrors = -1;
    if( aBaselineDrc.count >= 0 && aLiveDrc.count >= 0 )
    {
        introducedDrcErrors = 0;
        for( const auto& [key, count] : aLiveDrc.fingerprints )
        {
            const auto baseline = aBaselineDrc.fingerprints.find( key );
            introducedDrcErrors += std::max( 0, count - ( baseline == aBaselineDrc.fingerprints.end()
                                                                  ? 0
                                                                  : baseline->second ) );
        }
    }

    std::ostringstream json;
    json << "{\n"
         << "  \"board\": \"" << jsonEscape( aBoardPath ) << "\",\n"
         << "  \"host_validated\": " << ( aResult.hostValidated ? "true" : "false" ) << ",\n"
         << "  \"host_unconnected\": " << aResult.hostUnconnected << ",\n"
         << "  \"host_new_drc_violations\": " << aResult.hostNewDrcViolations << ",\n"
         << "  \"host_repair_passes\": " << aResult.hostRepairPasses << ",\n"
         << "  \"host_validation_ms\": " << aResult.hostValidationMilliseconds << ",\n"
         << "  \"host_session_ms\": " << aHostSessionMs << ",\n"
         << "  \"baseline_kicad_unconnected\": " << aBaselineUnconnected << ",\n"
         << "  \"kicad_unconnected\": " << aUnconnected << ",\n"
         << "  \"electrically_complete\": " << ( aUnconnected == 0 ? "true" : "false" ) << ",\n"
         << "  \"validation_complete\": " << ( aLiveDrc.count >= 0 ? "true" : "false" ) << ",\n"
         << "  \"total_connections\": " << metrics.totalConnections << ",\n"
         << "  \"routed_connections\": " << metrics.routedConnections << ",\n"
         << "  \"unrouted_connections\": " << metrics.unroutedConnections << ",\n"
         << "  \"completion_percent\": " << metrics.completionPercent << ",\n"
         << "  \"drc_errors\": " << metrics.drcViolations << ",\n"
         << "  \"baseline_kicad_drc_errors\": " << aBaselineDrc.count << ",\n"
         << "  \"kicad_drc_errors\": " << aLiveDrc.count << ",\n"
         << "  \"new_kicad_drc_errors\": "
         << introducedDrcErrors
         << ",\n"
         << "  \"baseline_kicad_drc_by_code\": {";

    bool firstCode = true;
    for( const auto& [code, count] : aBaselineDrc.byCode )
    {
        if( !firstCode )
            json << ", ";

        json << "\"" << code << "\": " << count;
        firstCode = false;
    }

    json << "},\n"
         << "  \"kicad_drc_by_code\": {";

    firstCode = true;
    for( const auto& [code, count] : aLiveDrc.byCode )
    {
        if( !firstCode )
            json << ", ";

        json << "\"" << code << "\": " << count;
        firstCode = false;
    }

    json << "},\n"
         << "  \"kicad_drc_details\": [";

    for( std::size_t index = 0; index < aLiveDrc.details.size(); ++index )
    {
        if( index != 0 )
            json << ", ";

        json << "\"" << jsonEscape( aLiveDrc.details[index] ) << "\"";
    }

    json << "],\n"
         << "  \"via_positions\": [";

    for( std::size_t index = 0; index < aResult.vias.size(); ++index )
    {
        if( index != 0 )
            json << ", ";

        const KICAD_AUTOROUTER::ROUTING_VIA& via = aResult.vias[index];
        json << "{\"x\": " << via.position.x << ", \"y\": " << via.position.y
             << ", \"net\": " << via.netCode << ", \"drill\": " << via.drill << "}";
    }

    json << "],\n"
         << "  \"track_length_mm\": " << metrics.routedLengthIU / 1000000.0 << ",\n"
         << "  \"airline_length_mm\": " << metrics.airlineLengthIU / 1000000.0 << ",\n"
         << "  \"via_count\": " << metrics.viaCount << ",\n"
         << "  \"runtime_ms\": " << aRuntimeMs << ",\n"
         << "  \"ripups\": " << metrics.ripups << ",\n"
         << "  \"routing_passes\": " << metrics.passes << ",\n"
         << "  \"optimization_passes\": " << metrics.optimizationPasses << ",\n"
         << "  \"expanded_nodes\": " << metrics.expandedNodes << ",\n"
         << "  \"segments\": " << metrics.segmentCount << ",\n"
         << "  \"fanout_connections\": " << metrics.fanoutConnections << ",\n"
         << "  \"complete\": "
         << ( !aResult.cancelled && aUnconnected == 0 && introducedDrcErrors == 0
                      ? "true" : "false" ) << ",\n"
         << "  \"worker_complete\": " << ( aResult.complete ? "true" : "false" ) << ",\n"
         << "  \"cancelled\": " << ( aResult.cancelled ? "true" : "false" ) << ",\n"
         << "  \"message\": \"" << jsonEscape( aResult.message ) << "\"\n"
         << "}\n";

    if( aPath.empty() )
    {
        std::cout << json.str();
        return;
    }

    std::ofstream output( aPath );
    if( !output )
        throw std::runtime_error( "cannot open output file: " + aPath );

    output << json.str();
}

} // namespace


int main( int argc, char** argv )
{
    bool wxStarted = false;
    bool pgmStarted = false;

    try
    {
        const OPTIONS options = parseOptions( argc, argv );
        if( !options.saveBoard.empty() && std::filesystem::exists( options.saveBoard ) )
            throw std::runtime_error( "--save-board refuses to overwrite an existing file" );
        wxStarted = wxInitialize( argc, argv );
        if( !wxStarted )
            throw std::runtime_error( "wxWidgets initialization failed" );

        SetPgm( &g_program );
        Pgm().InitPgm( true, true );
        pgmStarted = true;
        PROPERTY_MANAGER::Instance().Rebuild();

        auto cleanup = []( int aCode )
        {
            Pgm().Destroy();
            wxUninitialize();
            return aCode;
        };

        std::unique_ptr<BOARD> board = KI_TEST::ReadBoardFromFileOrStream( options.board );
        if( !board )
        {
            std::cerr << "failed to load board: " << options.board << '\n';
            return cleanup( 2 );
        }

        board->BuildListOfNets();
        if( options.stripTracks )
        {
            std::vector<PCB_TRACK*> tracks( board->Tracks().begin(), board->Tracks().end() );
            for( PCB_TRACK* track : tracks )
            {
                if( track->IsLocked() )
                    continue;
                board->Remove( track );
                delete track;
            }
        }
        board->BuildConnectivity();

        // Board files loaded by the QA utility intentionally do not own a
        // DRC engine.  Install the same engine/constraint resolver that the
        // editor uses before creating the snapshot; this makes pair-specific
        // clearance capture and the optional post-route live DRC meaningful.
        auto drcEngine = std::make_shared<DRC_ENGINE>(
                board.get(), &board->GetDesignSettings() );
        wxFileName rules( options.board );
        rules.SetExt( "kicad_dru" );
        drcEngine->InitEngine( rules.Exists() ? rules : wxFileName() );
        board->GetDesignSettings().m_DRCEngine = std::move( drcEngine );

        // Pour geometry must describe the actual copper after optional strip,
        // and both engines must receive this same freshly filled input.
        if( !board->Zones().empty() && !ZONE_FILLER( board.get(), nullptr ).Fill( board->Zones() ) )
            throw std::runtime_error( "input zone refill failed" );
        board->BuildConnectivity();
        if( !options.exportDsn.empty() )
        {
            DSN::ExportBoardToSpecctraFile(
                    board.get(), wxString::FromUTF8( options.exportDsn.c_str() ) );
            if( !options.saveBoard.empty() )
                KI_TEST::DumpBoardToFile( *board, options.saveBoard );
            return cleanup( 0 );
        }

        KICAD_AUTOROUTER::KICAD_BOARD_ADAPTER adapter( board.get() );
        KICAD_AUTOROUTER::AUTOROUTER_SETTINGS settings = adapter.CreateDefaultSettings();
        settings.maxPasses = options.maxPasses;
        settings.maxIterations = options.maxIterations;
        settings.optimizationPasses = options.optimizationPasses;
        settings.maxExpandedNodes = options.maxExpandedNodes;
        settings.enableFanout = options.fanout;
        settings.allowVias = options.vias;
        settings.routeOnlyUnconnected = options.routeOnlyUnconnected;
        settings.includeNets = options.includeNets;

        // Large corpus boards are useful for full runs but expensive to use
        // as a quick QA smoke test.  Select a stable prefix of board nets
        // before snapshot capture so the adapter and worker exercise the same
        // filtering path as the editor settings dialog.
        if( options.maxNets > 0 && settings.includeNets.empty() )
        {
            for( NETINFO_ITEM* netInfo : board->GetNetInfo() )
            {
                if( !netInfo || netInfo->GetNetCode() <= 0 )
                    continue;

                settings.includeNets.emplace_back( netInfo->GetNetname().ToUTF8() );
                if( static_cast<int>( settings.includeNets.size() ) >= options.maxNets )
                    break;
            }
        }

        std::shared_ptr<const KICAD_AUTOROUTER::BOARD_SNAPSHOT> snapshot =
                adapter.CreateSnapshot( settings );
        if( !snapshot )
        {
            std::cerr << "failed to create board snapshot\n";
            return cleanup( 2 );
        }

        if( options.maxConnections > 0 )
        {
            // QA-only bounded diagnosis for large multi-pad or plane nets.
            // The editor never truncates a connection graph; this makes it
            // possible to inspect a few independent connection items without
            // waiting for a complete dense-board run.
            auto boundedSnapshot =
                    std::make_shared<KICAD_AUTOROUTER::BOARD_SNAPSHOT>( *snapshot );
            for( KICAD_AUTOROUTER::ROUTING_NET& net : boundedSnapshot->nets )
            {
                if( net.connections.size() > static_cast<std::size_t>( options.maxConnections ) )
                {
                    net.connections.resize(
                            static_cast<std::size_t>( options.maxConnections ) );
                }
            }
            snapshot = std::move( boundedSnapshot );
        }

        if( options.dumpSnapshot )
        {
            std::cerr << "snapshot bounds=" << snapshot->bounds.minX << ','
                      << snapshot->bounds.minY << ".." << snapshot->bounds.maxX << ','
                      << snapshot->bounds.maxY << " layers=" << settings.layers.size()
                      << " pads=" << snapshot->pads.size()
                      << " obstacles=" << snapshot->obstacles.size() << " nets="
                      << snapshot->nets.size() << "\n";
            for( const auto& net : snapshot->nets )
            {
                std::cerr << "  net " << net.netCode << " '" << net.name << "' pads="
                          << net.padIndices.size() << " connections=" << net.connections.size()
                          << " routable=" << net.routable << "\n";
                for( const auto& connection : net.connections )
                {
                    if( connection.first < snapshot->pads.size()
                        && connection.second < snapshot->pads.size() )
                    {
                        const auto& first = snapshot->pads[connection.first];
                        const auto& second = snapshot->pads[connection.second];
                        std::cerr << "    " << connection.first << "@" << first.position.x << ','
                                  << first.position.y << " -> " << connection.second << "@"
                                  << second.position.x << ',' << second.position.y << "\n";
                    }
                }
            }
            for( std::size_t index = 0; index < snapshot->pads.size(); ++index )
            {
                const auto& pad = snapshot->pads[index];
                std::cerr << "  pad " << index << " net=" << pad.netCode << " @"
                          << pad.position.x << ',' << pad.position.y << " layers=";
                for( int layer : pad.layers )
                    std::cerr << layer << ',';
                std::cerr << " smd=" << pad.isSmd << " plane=" << pad.isPlaneTarget
                          << " radius=" << pad.radius << " clearance=" << pad.clearance << "\n";
            }

            for( std::size_t index = 0; index < snapshot->obstacles.size(); ++index )
            {
                const auto& obstacle = snapshot->obstacles[index];
                if( !obstacle.isHole )
                    continue;

                std::cerr << "  hole " << index << " net=" << obstacle.netCode << " @"
                          << obstacle.start.x << ',' << obstacle.start.y
                          << " existing=" << obstacle.isExistingRoute << " id='"
                          << obstacle.boardItemId << "' layers=";
                for( int layer : obstacle.layers )
                    std::cerr << layer << ',';
                std::cerr << "\n";
            }

        }

        LIVE_DRC_RESULT baselineDrc;
        board->GetConnectivity()->RecalculateRatsnest();
        const int baselineUnconnected = board->GetConnectivity()->GetUnconnectedCount( false );
        if( options.liveDrc )
            baselineDrc = runLiveDrc( board.get() );

        const auto start = std::chrono::steady_clock::now();
        KICAD_AUTOROUTER::ROUTING_RESULT result;
        if( options.importSes.empty() )
        {
            if( options.maxConnections > 0 ) // explicitly bounded raw-worker diagnosis
                result = KICAD_AUTOROUTER::ROUTING_PIPELINE().Run( *snapshot, settings, {}, {} );
            else
                result = KICAD_AUTOROUTER::KICAD_ROUTING_SESSION( *board ).Run( settings );
        }
        else
        {
            REFERENCE_IMPORT_COMMIT commit( *board );
            if( !DSN::ImportSpecctraSession( board.get(), wxString::FromUTF8( options.importSes ),
                                            commit ) )
                throw std::runtime_error( "reference session import failed" );
            commit.Push( wxEmptyString, 0 );
            result.message = "Reference session evaluated by KiCad";
        }
        const auto end = std::chrono::steady_clock::now();
        const auto runtimeMs = result.hostValidated
                ? result.metrics.elapsedMilliseconds - result.hostValidationMilliseconds
                : std::chrono::duration_cast<std::chrono::milliseconds>( end - start ).count();
        const auto hostSessionMs = result.hostValidated
                ? std::chrono::duration_cast<std::chrono::milliseconds>( end - start ).count() : -1;

        LIVE_DRC_RESULT liveDrc;
        applyProposal( board.get(), result, adapter );
        const int unconnected = board->GetConnectivity()->GetUnconnectedCount( false );
        // Measure physical copper for either engine with the same host API.
        // Do not compare native synthetic connection counts with Java items.
        result.metrics.viaCount = 0;
        result.metrics.routedLengthIU = 0;
        for( PCB_TRACK* track : board->Tracks() )
        {
            if( track->Type() == PCB_VIA_T )
                ++result.metrics.viaCount;
            else
                result.metrics.routedLengthIU += track->GetLength();
        }
        if( options.liveDrc && !result.cancelled )
            liveDrc = runLiveDrc( board.get() );

        writeJson( options.output, options.board, result, baselineDrc, liveDrc, runtimeMs, hostSessionMs,
                   baselineUnconnected, unconnected );
        if( !options.saveBoard.empty() )
            KI_TEST::DumpBoardToFile( *board, options.saveBoard );
        return cleanup( result.cancelled ? 3 : 0 );
    }
    catch( const std::exception& exception )
    {
        std::cerr << exception.what() << '\n';
        if( pgmStarted )
            Pgm().Destroy();
        if( wxStarted )
            wxUninitialize();
        return 2;
    }
}
