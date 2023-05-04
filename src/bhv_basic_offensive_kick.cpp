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

#include "bhv_basic_offensive_kick.h"

#include <rcsc/action/body_hold_ball.h>
#include <rcsc/action/body_smart_kick.h>
#include <rcsc/action/body_stop_ball.h>
#include <rcsc/player/player_agent.h>
#include <rcsc/player/debug_client.h>

#include <rcsc/common/logger.h>
#include <rcsc/common/server_param.h>

#include <rcsc/geom/sector_2d.h>

#include <vector>
using namespace rcsc;

/*-------------------------------------------------------------------*/
/*!

 */


bool check_space(const WorldModel & wm){
    AngleDeg base = wm.self().vel().th().degree(); //Vector2D(52.5,0).th().degree();
    Sector2D goal_sect = Sector2D(wm.self().pos(), 0, 8, base + AngleDeg(-45), base + AngleDeg(45));
    Sector2D back_sect = Sector2D(wm.self().pos(), 0, 6, base + AngleDeg(45), base + AngleDeg(-45));
    if (wm.existOpponentIn(goal_sect,8,false) || 
        wm.existOpponentIn(back_sect,8,false)){
        return true;
    }
    if (wm.existTeammateIn(goal_sect,6,false)){
        return true;
    }
    return false;
}

Vector2D point_proj(Vector2D & lp1, Vector2D & lp2, Vector2D & p){
    double t = ((p.x - lp1.x) * (lp2.x - lp1.x) + 
            (p.y - lp1.y) * (lp2.y - lp1.y)) / 
            (pow((lp2.x - lp1.x), 2) + 
            pow((lp2.y - lp1.y), 2));
    if(t < 0 || t > 1){
        return {100, 100};
    }

    double xc = lp1.x + t * (lp2.x - lp1.x);
    double yc = lp1.y + t * (lp2.y - lp1.y);
    
    return {xc, yc};
}
    
double check_interf(const WorldModel & player, Vector2D & target, const AbstractPlayerObject * op, double ball_speed){
    Vector2D p_pos = player.self().pos();
    Vector2D op_pos = op->pos();
    const PlayerType * op_type = op->playerTypePtr();

    Vector2D projection_point = point_proj(p_pos, target, op_pos);
    
    if (projection_point != Vector2D(100, 100)){
        double op_dist = op_pos.dist(projection_point);
        double min_proj_cycle = p_pos.dist(projection_point) / ball_speed;
        double min_op_cycle = op_dist / op_type->realSpeedMax();
        
        if (op_dist < 0.5 && min_proj_cycle >= 1) {return -100.0;};

        dlog.addText(Logger::BLOCK, "---------> Min reach is %.2f", min_proj_cycle);
        dlog.addText(Logger::BLOCK, "---------> Min cycle is %.2f", min_op_cycle);

        return(min_op_cycle - min_proj_cycle);
    }

    return(1);
}

double check_interf(const AbstractPlayerObject & player, Vector2D & target, const AbstractPlayerObject * op, double ball_speed){
    Vector2D p_pos = player.pos();
    Vector2D op_pos = op->pos();
    const PlayerType * op_type = op->playerTypePtr();

    Vector2D projection_point = point_proj(p_pos, target, op_pos);
    
    if (projection_point != Vector2D(100, 100)){
        double op_dist = op_pos.dist(projection_point);
        double min_proj_cycle = p_pos.dist(projection_point) / ball_speed;
        double min_op_cycle = op_dist / op_type->realSpeedMax();
        
        if (op_dist < 0.5 && min_proj_cycle >= 1) {return -100.0;};

        dlog.addText(Logger::BLOCK, "---------> Min reach is %.2f", min_proj_cycle);
        dlog.addText(Logger::BLOCK, "---------> Min cycle is %.2f", min_op_cycle);

        return(min_op_cycle - min_proj_cycle);
    }

    return(1);
}

bool Bhv_BasicOffensiveKick::possible_pass(PlayerAgent * agent, Vector2D & target, const int & cycle_thr){
    const WorldModel & wm = agent->world();
    int min_op_reach = 100;
    for (int i = 1; i <= 11; i++){
        const AbstractPlayerObject * op = wm.theirPlayer(i);
        if(op == NULL || op->unum() < 1)
            continue;
        double reach = check_interf(wm, target, op, 2);
        min_op_reach = reach < min_op_reach ? reach : min_op_reach;
    }
    if (min_op_reach + cycle_thr > 0) {
        return true;
    } else {
        return false;
    }
    
}

Vector2D Bhv_BasicOffensiveKick::best_shoot_place(PlayerAgent * agent, const int & cycle_thr){
    Vector2D target;
    Vector2D best_target;
    double best_reach = -100;
    const WorldModel & wm = agent->world();
    for (double i = -6; i < 7; i++){
        target = {52, i};
        int ops_reach = 100;
        for (int i = 1; i <= 11; i++){
            const AbstractPlayerObject * op = wm.theirPlayer(i);
            if(op == NULL || op->unum() < 1)
                continue;
            double reach = check_interf(wm, target, op, 2);
            ops_reach = reach < ops_reach ? reach : ops_reach;
        }
        if (ops_reach > best_reach && ops_reach != 100){
            best_target = target;
            best_reach = ops_reach;
        }
    }

    if (best_reach + cycle_thr > 0)
        return best_target;
    else
        return Vector2D(100, 100);
}

bool
Bhv_BasicOffensiveKick::execute( PlayerAgent * agent )
{
    dlog.addText( Logger::TEAM,
                  __FILE__": Bhv_BasicOffensiveKick" );

    const WorldModel & wm = agent->world();

    if(shoot(agent)){
    	return true;
    }

    const PlayerPtrCont & opps = wm.opponentsFromSelf();
    const PlayerObject * nearest_opp
        = ( opps.empty()
            ? static_cast< PlayerObject * >( 0 )
            : opps.front() );
    const double nearest_opp_dist = ( nearest_opp
                                      ? nearest_opp->distFromSelf()
                                      : 1000.0 );

    if(check_space(wm)){
    	if(pass(agent))
    		return true;
    }

    if(dribble(agent)){
    	return true;
    }

    if ( nearest_opp_dist > 2.5 )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": hold" );
        agent->debugClient().addMessage( "OffKickHold" );
        Body_HoldBall().execute( agent );
        return true;
    }
    clearball(agent);
    return true;
}

bool Bhv_BasicOffensiveKick::shoot( rcsc::PlayerAgent * agent ){
	const WorldModel & wm = agent->world();
	Vector2D ball_pos = wm.ball().pos();
	Vector2D center_goal = Vector2D(52.5,0);
	if(ball_pos.dist(center_goal) > 25)
            return false;
    Vector2D best_shoot = best_shoot_place(agent, 2);

    if (best_shoot != Vector2D(100, 100)){
        if (!Body_SmartKick(best_shoot,3,2,2).execute(agent)){
            Body_StopBall().execute(agent);
        }
        return true;
    }

	return false;
}

bool Bhv_BasicOffensiveKick::pass(PlayerAgent * agent){
	const WorldModel & wm = agent->world();
	std::vector<Vector2D> targets;
	Vector2D ball_pos = wm.ball().pos();
	for(int u = 1;u<=11;u++){
		const AbstractPlayerObject * tm = wm.ourPlayer(u);
		if(tm==NULL || tm->unum() < 1 || tm->unum() == wm.self().unum() )
			continue;
		Vector2D tm_pos = tm->pos();
		if(tm->pos().dist(ball_pos) > 30)
			continue;
		if( possible_pass(agent, tm_pos, 2) ){
			targets.push_back(tm_pos);
		}
	}
	if(targets.size() == 0)
		return false;
	Vector2D best_target = targets[0];
    for(unsigned int i=1;i<targets.size();i++){
		if(targets[i].x > best_target.x)
			best_target = targets[i];
	}
	if(wm.gameMode().type()!= GameMode::PlayOn)
        Body_SmartKick(best_target,3,2.5,1).execute(agent);
	else
        Body_SmartKick(best_target,3,2.5,2).execute(agent);
	return true;
}

bool Bhv_BasicOffensiveKick::dribble(PlayerAgent * agent){
	const WorldModel & wm = agent->world();
	Vector2D ball_pos = wm.ball().pos();
	double dribble_angle = (Vector2D(52.5,0) - ball_pos).th().degree();
	Sector2D dribble_sector = Sector2D(ball_pos,0,3,dribble_angle - 15,dribble_angle+15);
	if(!wm.existOpponentIn(dribble_sector,5,true)){
		Vector2D target = Vector2D::polar2vector(3,dribble_angle) + ball_pos;
		if(Body_SmartKick(target,0.8,0.7,2).execute(agent)){
			return true;
		}
	}
	return false;
}

bool Bhv_BasicOffensiveKick::clearball(PlayerAgent * agent){
    const WorldModel & wm = agent->world();
    if(!wm.self().isKickable())
        return false;
    Vector2D ball_pos = wm.ball().pos();
    Vector2D target = Vector2D(52.5,0);
    if(ball_pos.x < 0){
        if(ball_pos.x > -25){
            if(ball_pos.dist(Vector2D(0,-34)) < ball_pos.dist(Vector2D(0,+34))){
                target = Vector2D(0,-34);
            }else{
                target = Vector2D(0,+34);
            }
        }else{
            if(ball_pos.absY() < 10 && ball_pos.x < -10){
                if(ball_pos.y > 0){
                    target = Vector2D(-52,20);
                }else{
                    target = Vector2D(-52,-20);
                }
            }else{
                if(ball_pos.y > 0){
                    target = Vector2D(ball_pos.x,34);
                }else{
                    target = Vector2D(ball_pos.x,-34);
                }
            }
        }
    }
    if(Body_SmartKick(target,2.7,2.7,2).execute(agent)){
        return true;
    }
    Body_StopBall().execute(agent);
    return true;
}
