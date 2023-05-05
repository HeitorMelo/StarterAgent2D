// -*-c++-*-

/*
 *Copyright:

 Copyright (C) Hidehisa AKIYAMA

 This code is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 3, or (at your option)
 any later version.

 This code is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this code; see the file COPYING.  If not, write to
 the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.

 *EndCopyright:
 */

//Student Soccer 2D Simulation Base , STDAGENT2D
//Simplified the Agent2D Base for HighSchool Students.
//Technical Committee of Soccer 2D Simulation League, IranOpen
//Nader Zare
//Mostafa Sayahi
//Pooria Kaviani
/////////////////////////////////////////////////////////////////////

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "bhv_basic_move.h"

#include "bhv_basic_tackle.h"
#include "bhv_basic_offensive_kick.h"
#include <rcsc/action/neck_turn_to_player_or_scan.h>

#include <rcsc/action/basic_actions.h>
#include <rcsc/action/body_go_to_point.h>
#include <rcsc/action/body_intercept.h>

#include <rcsc/player/player_agent.h>
#include <rcsc/player/debug_client.h>
#include <rcsc/player/intercept_table.h>
#include <rcsc/player/player_intercept.h>

#include <rcsc/common/logger.h>
#include <rcsc/common/server_param.h>

#include <vector>
#include <cstdio>

using namespace rcsc;

/*-------------------------------------------------------------------*/
/*!

 */


std::vector<Vector2D> createBallCache(PlayerAgent * agent)
{
    std::vector<Vector2D> M_ball_pos_cache;

    const WorldModel & wm = agent->world();
    const ServerParam & SP = ServerParam::i();
    const double pitch_x_max = ( SP.keepawayMode()
                                 ? SP.keepawayLength() * 0.5
                                 : SP.pitchHalfLength() + 5.0 );
    const double pitch_y_max = ( SP.keepawayMode()
                                 ? SP.keepawayWidth() * 0.5
                                 : SP.pitchHalfWidth() + 5.0 );
    const double bdecay = SP.ballDecay();

    Vector2D bpos = wm.ball().pos();
    Vector2D bvel = wm.ball().vel();

    M_ball_pos_cache.push_back( bpos );

    for (int i = 1; i <= 30; ++i )
    {
        bpos += bvel;
        bvel *= bdecay;

        M_ball_pos_cache.push_back( bpos );

        if ( i >= 5
             && bvel.r2() < 0.01*0.01 )
        {
            // ball stopped
            break;
        }

        if ( bpos.absX() > pitch_x_max
             || bpos.absY() > pitch_y_max )
        {
            // out of pitch
            break;
        }
    }

    if ( M_ball_pos_cache.size() == 1 )
    {
        M_ball_pos_cache.push_back( bpos );
    }
    
    return M_ball_pos_cache;
}


bool intention(PlayerAgent * agent){
    const WorldModel & wm = agent->world();
    dlog.addText(Logger::BLOCK, "Starting Intention");

    PlayerObject *teammate = NULL, *opponent = NULL;
    bool tm_reaching, op_reaching;
    Vector2D ball_pos;

    const PlayerPtrCont::const_iterator tm_end = wm.teammatesFromBall().end();
    for ( PlayerPtrCont::const_iterator it = wm.teammatesFromBall().begin();
          it != tm_end;
          ++it)
    {
        if ((*it)->posCount() >= 10 ||
            (*it)->unum() == wm.self().unum())
            continue;
        
        ball_pos = wm.ball().pos();
        tm_reaching = ball_pos.dist((*it)->inertiaPoint(2)) < ball_pos.dist((*it)->pos());
        if (!((*it)->playerTypePtr()))
            continue;
        if (tm_reaching){
            teammate = *it;
            break;
        }
    }

    const PlayerPtrCont::const_iterator op_end = wm.opponentsFromBall().end();
    for ( PlayerPtrCont::const_iterator it = wm.opponentsFromBall().begin();
          it != op_end;
          ++it)
    {
        if ((*it)->posCount() >= 10)
            continue;
        
        ball_pos = wm.ball().pos();
        op_reaching = ball_pos.dist((*it)->inertiaPoint(2)) < ball_pos.dist((*it)->pos());
        if (!((*it)->playerTypePtr()))
            continue;
        if (op_reaching){
            opponent = *it;
            break;
        }
    }

    if((teammate == NULL || opponent == NULL) && wm.self().distFromBall() > 10){
        dlog.addText(Logger::BLOCK, "Found a NULL ----------------");
        return false;
    }

    const std::vector<Vector2D> Ball_cache = createBallCache(agent);
    if (Ball_cache.empty()){
        dlog.addText(Logger::BLOCK, "Ball_cache empty ----------------");
        return false;
    }
    PlayerIntercept predictor = PlayerIntercept(wm, Ball_cache);
    
    int self_min = wm.interceptTable()->selfReachCycle();
    int tm_min = 1000;  
    int op_min = 1000;

    if (teammate){
        const PlayerType * tm_type = teammate->playerTypePtr();
        if (tm_type)
            tm_min = predictor.predict( *teammate, *tm_type, Ball_cache.size() );
    } 

    if (opponent){
        const PlayerType * op_type = opponent->playerTypePtr();
        if (op_type)
            op_min = predictor.predict( *opponent, *op_type, Ball_cache.size() );
    }

    if ( (self_min <= 4 && wm.lastKickerSide() == wm.ourSide()) || 
        (self_min <= tm_min && self_min < op_min + 3)){
        dlog.addText(Logger::BLOCK, "Intention returning True ----------------");
        return true;
    } 

    dlog.addText(Logger::BLOCK, "Intention returning False ----------------");
    return false;
}


void Bhv_BasicMove::check_players(PlayerAgent * agent){
    const WorldModel & wm = agent->world();
    Vector2D self_pos = wm.self().pos();

    //viewWidth
    const int opp_min = wm.interceptTable()->opponentReachCycle();
    const int self_min = wm.interceptTable()->selfReachCycle();
    const int mate_min = wm.interceptTable()->teammateReachCycle();
    
    if (wm.ball().seenPosCount() >= self_min - 1 || (self_min <= 5 && opp_min <= 6))
        agent->doChangeView( ViewWidth::NARROW);
    else if (self_min <= 15)
        agent->doChangeView( ViewWidth::NORMAL );
    else if (!wm.ball().posValid() || (self_min > mate_min && opp_min < self_min))
        agent->doChangeView( ViewWidth::WIDE );
    else
        agent->doChangeView( ViewWidth::NORMAL );   
    

    //check positions for possible mate pass
    std::vector<PlayerObject *> teams; 
    std::vector<PlayerObject *> oppons;


    const PlayerPtrCont::const_iterator tm_end = wm.teammatesFromSelf().end();
    for (PlayerPtrCont::const_iterator tm_player = wm.teammatesFromSelf().begin();
        tm_player != tm_end; 
        ++tm_player)
    {
        if ( *tm_player == NULL ||
            (*tm_player)->isGhost() ||
            (*tm_player)->unum() == wm.self().unum() ||
            ((*tm_player)->distFromSelf() > 30 )
        )
            continue; 
        if ((*tm_player)->pos().absX() > self_pos.absX() - 10)
            teams.push_back(*tm_player);
    }
    
    const PlayerPtrCont::const_iterator op_end = wm.opponentsFromSelf().end();
    for (PlayerPtrCont::const_iterator op_player = wm.opponentsFromSelf().begin();
        op_player != op_end; 
        ++op_player)
    {
        if ( *op_player == NULL ||
            (*op_player)->isGhost() ||
            (*op_player)->distFromSelf() > 25
        )
            continue;
        if ((*op_player)->pos().absX() > self_pos.absX() - 10)
            oppons.push_back(*op_player);
    }

    if (wm.existKickableTeammate()){
        const PlayerObject * nearest_tm = wm.getTeammateNearestToBall(5, false);
        if (nearest_tm != NULL ){
            Vector2D tm_pos = nearest_tm->pos();
            
            bool poss_pass = Bhv_BasicOffensiveKick().possible_pass(agent, tm_pos, 5);
            if (poss_pass && nearest_tm->posCount() >= 2){
                if (wm.self().seenPosCount() == 0)
                    agent->doChangeView( ViewWidth::NARROW );
                else if (wm.self().seenPosCount() == 1)
                    agent->doChangeView( ViewWidth::NORMAL );
                else
                    agent->doChangeView( ViewWidth::WIDE );
                Bhv_NeckBodyToBall().execute(agent);
                return;
            }
        }
    }
    
    
    int last_tm_seen = 100;
    PlayerObject * last_tm = NULL;
    int last_op_seen = 100;
    PlayerObject * last_op = NULL;
    for (PlayerObject * tm: teams){
        if (last_tm_seen > tm->seenPosCount()){
            last_tm_seen = tm->seenPosCount();
            last_tm = tm;
        }
    }

    for (PlayerObject * op: oppons){
        if (last_op_seen > op->seenPosCount()){
            last_op_seen = op->seenPosCount();
            last_op = op;
        }
    }

    int goalie = wm.theirGoalieUnum();
    if (wm.self().pos().dist(Vector2D(52.5,0)) < 20 && goalie != Unum_Unknown){
        if (wm.theirPlayer(goalie)->posCount() > 3){
            Neck_TurnToPoint((Vector2D(52.5, 0))).execute(agent);
            return;
        }
    }

    if (wm.ball().seenPosCount() >= 3){
        Bhv_NeckBodyToBall().execute(agent);
        return;
    }
    const PlayerObject * near_op = wm.getOpponentNearestToSelf(5, false);
    if (last_op != NULL && near_op != NULL){
        if (near_op->pos().dist(wm.self().pos()) > 10) {
            Neck_TurnToPlayerOrScan(wm.theirPlayer(last_op->unum())).execute(agent);
            return;
        }
    } 
    if (last_tm != NULL){
        if ( last_tm->unum() >= 1) {
            Neck_TurnToPlayerOrScan(wm.ourPlayer(last_tm->unum())).execute(agent);
            return;
        }
    }

    
    Bhv_NeckBodyToBall().execute(agent);

}


bool
Bhv_BasicMove::execute( PlayerAgent * agent )
{
    dlog.addText( Logger::TEAM,
                  __FILE__": Bhv_BasicMove" );

    //-----------------------------------------------
    // tackle
    if ( Bhv_BasicTackle( 0.8, 80.0 ).execute( agent ) )
    {
        return true;
    }

    const WorldModel & wm = agent->world();
    /*--------------------------------------------------------*/
    // chase ball
    const int self_min = wm.interceptTable()->selfReachCycle();
    const int mate_min = wm.interceptTable()->teammateReachCycle();
    const int opp_min = wm.interceptTable()->opponentReachCycle();

    check_players(agent);
    if ( ! wm.existKickableTeammate()
         && ( self_min <= 3
              || intention(agent)
              )
         )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": intercept" );
        Body_Intercept().execute( agent );

        return true;
    }

    const Vector2D target_point = getPosition( wm, wm.self().unum() );
    const double dash_power = get_normal_dash_power( wm );

    double dist_thr = wm.ball().distFromSelf() * 0.1;
    if ( dist_thr < 1.0 ) dist_thr = 1.0;

    dlog.addText( Logger::TEAM,
                  __FILE__": Bhv_BasicMove target=(%.1f %.1f) dist_thr=%.2f",
                  target_point.x, target_point.y,
                  dist_thr );

    agent->debugClient().addMessage( "BasicMove%.0f", dash_power );
    agent->debugClient().setTarget( target_point );
    agent->debugClient().addCircle( target_point, dist_thr );

    if ( ! Body_GoToPoint( target_point, dist_thr, dash_power
                           ).execute( agent ) )
    {
        Body_TurnToBall().execute( agent );
    }

    return true;
}

rcsc::Vector2D Bhv_BasicMove::getPosition(const rcsc::WorldModel & wm, int self_unum){
    int ball_step = 0;
    if ( wm.gameMode().type() == GameMode::PlayOn
         || wm.gameMode().type() == GameMode::GoalKick_ )
    {
        ball_step = std::min( 1000, wm.interceptTable()->teammateReachCycle() );
        ball_step = std::min( ball_step, wm.interceptTable()->opponentReachCycle() );
        ball_step = std::min( ball_step, wm.interceptTable()->selfReachCycle() );
    }

    Vector2D ball_pos = wm.ball().inertiaPoint( ball_step );

    dlog.addText( Logger::TEAM,
                  __FILE__": HOME POSITION: ball pos=(%.1f %.1f) step=%d",
                  ball_pos.x, ball_pos.y,
                  ball_step );

    std::vector<Vector2D> positions(12);
    double min_x_rectengle[12]={0,-52,-52,-52,-52,-52,-30,-30,-30,0,0,0};
    double max_x_rectengle[12]={0,-48,-10,-10,-10,-10,15,15,15,50,50,50};
    double min_y_rectengle[12]={0,-2,-20,-10,-30,10,-20,-30, 0,-20,-30, 0};
    double max_y_rectengle[12]={0,+2, 10, 20,-10,30, 20, 0,30, 20, 0, 30};

    for(int i=1; i<=11; i++){
          double xx_rectengle = max_x_rectengle[i] - min_x_rectengle[i];
          double yy_rectengle = max_y_rectengle[i] - min_y_rectengle[i];
          double x_ball = ball_pos.x + 52.5;
          x_ball /= 105.5;
          double y_ball = ball_pos.y + 34;
          y_ball /= 68.0;
          double x_pos = xx_rectengle * x_ball + min_x_rectengle[i];
          double y_pos = yy_rectengle * y_ball + min_y_rectengle[i];
          positions[i] = Vector2D(x_pos,y_pos);
    }

    if ( ServerParam::i().useOffside() )
    {
        double max_x = wm.offsideLineX();
        if ( ServerParam::i().kickoffOffside()
             && ( wm.gameMode().type() == GameMode::BeforeKickOff
                  || wm.gameMode().type() == GameMode::AfterGoal_ ) )
        {
            max_x = 0.0;
        }
        else
        {
            int mate_step = wm.interceptTable()->teammateReachCycle();
            if ( mate_step < 50 )
            {
                Vector2D trap_pos = wm.ball().inertiaPoint( mate_step );
                if ( trap_pos.x > max_x ) max_x = trap_pos.x;
            }

            max_x -= 1.0;
        }

        for ( int unum = 1; unum <= 11; ++unum )
        {
            if ( positions[unum].x > max_x )
            {
                dlog.addText( Logger::TEAM,
                              "____ %d offside. home_pos_x %.2f -> %.2f",
                              unum,
                              positions[unum].x, max_x );
                positions[unum].x = max_x;
            }
        }
    }
    return positions.at(self_unum);
}

double Bhv_BasicMove::get_normal_dash_power( const WorldModel & wm )
{
    static bool s_recover_mode = false;

    if ( wm.self().staminaModel().capacityIsEmpty() )
    {
        return std::min( ServerParam::i().maxDashPower(),
                         wm.self().stamina() + wm.self().playerType().extraStamina() );
    }

    // check recover
    if ( wm.self().staminaModel().capacityIsEmpty() )
    {
        s_recover_mode = false;
    }
    else if ( wm.self().stamina() < ServerParam::i().staminaMax() * 0.5 )
    {
        s_recover_mode = true;
    }
    else if ( wm.self().stamina() > ServerParam::i().staminaMax() * 0.7 )
    {
        s_recover_mode = false;
    }

    /*--------------------------------------------------------*/
    double dash_power = ServerParam::i().maxDashPower();
    const double my_inc
        = wm.self().playerType().staminaIncMax()
        * wm.self().recovery();

    if ( wm.ourDefenseLineX() > wm.self().pos().x
         && wm.ball().pos().x < wm.ourDefenseLineX() + 20.0 )
    {
    }
    else if ( s_recover_mode )
    {
    }
    else if ( wm.existKickableTeammate()
              && wm.ball().distFromSelf() < 20.0 )
    {
    }
    else if ( wm.self().pos().x > wm.offsideLineX() )
    {
    }
    else
    {
        dash_power = std::min( my_inc * 1.7,
                               ServerParam::i().maxDashPower() );
    }

    return dash_power;
}

