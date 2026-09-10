/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <limits>

namespace KICAD_AUTOROUTER
{

/**
 * Score convergence state translated from Freerouting's AutorouteBatchLoop.
 * Board restoration remains a caller responsibility because this class does
 * not own worker snapshots.
 */
class AUTOROUTE_BATCH_LOOP
{
public:
    static constexpr int STOP_AT_PASS_MINIMUM = 8;
    static constexpr int STOP_AT_PASS_MODULO = 4;
    static constexpr int STAGNATION_PASS_LIMIT = 10;
    static constexpr int FANOUT_RECOVERY_STAGNATION_PASSES = 3;
    static constexpr double STAGNATION_SCORE_THRESHOLD = 0.5;

    struct DECISION
    {
        bool stop = false;
        bool recoverFanout = false;
    };

    DECISION Observe( int aPass, double aScore, int aIncompleteCount,
                      bool aContinueAutorouting, bool aFanoutEnabled )
    {
        DECISION result;
        if( aPass < STOP_AT_PASS_MINIMUM || !aContinueAutorouting )
        {
            if( aIncompleteCount == 0 && aScore > STAGNATION_SCORE_THRESHOLD )
            {
                m_consecutiveNoImprovementPasses = 0;
                m_lastBestScore = aScore;
            }
            return result;
        }

        if( aScore > m_lastBestScore + STAGNATION_SCORE_THRESHOLD )
        {
            m_consecutiveNoImprovementPasses = 0;
            m_lastBestScore = aScore;
        }
        else
        {
            ++m_consecutiveNoImprovementPasses;
            if( aFanoutEnabled && !m_fanoutRecoveryApplied && aIncompleteCount > 0
                && m_consecutiveNoImprovementPasses
                           >= FANOUT_RECOVERY_STAGNATION_PASSES )
            {
                m_fanoutRecoveryApplied = true;
                result.recoverFanout = true;
                return result;
            }

            if( m_consecutiveNoImprovementPasses >= STAGNATION_PASS_LIMIT )
                result.stop = true;
        }

        observeGlobal( aPass, aScore, result );
        return result;
    }

    /** Complete the source loop's one-time fanout-tail cleanup with the score
     * recalculated from the cleaned board. */
    DECISION ApplyFanoutRecoveryScore( int aPass, double aScore )
    {
        m_consecutiveNoImprovementPasses = 0;
        m_lastBestScore = aScore;
        DECISION result;
        observeGlobal( aPass, aScore, result );
        return result;
    }

    void RestoredBoard( double aScore )
    {
        m_consecutiveNoImprovementPasses = 0;
        m_lastBestScore = aScore;
    }

    int StagnantPasses() const { return m_consecutiveNoImprovementPasses; }
    int PassOfBestScore() const { return m_passOfBestScore; }
    double GlobalBestScore() const { return m_globalBestScore; }

private:
    void observeGlobal( int aPass, double aScore, DECISION& aDecision )
    {
        if( aScore > m_globalBestScore + STAGNATION_SCORE_THRESHOLD )
        {
            m_globalBestScore = aScore;
            m_passOfBestScore = aPass;
        }
        else if( aPass - m_passOfBestScore >= STAGNATION_PASS_LIMIT )
        {
            aDecision.stop = true;
        }
    }

    int    m_consecutiveNoImprovementPasses = 0;
    double m_lastBestScore = -std::numeric_limits<double>::infinity();
    double m_globalBestScore = -std::numeric_limits<double>::infinity();
    int    m_passOfBestScore = 0;
    bool   m_fanoutRecoveryApplied = false;
};

} // namespace KICAD_AUTOROUTER
