#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include "PAPF_Planner.h"

PAPF_Planner::PAPF_Planner(Params params) : m_params(params) {}

double PAPF_Planner::normalizeAngle(double angle) {
    return atan2(sin(angle), cos(angle));
}

Vector2D PAPF_Planner::calculateAttractiveForce(const USV& usv, const Vector2D& goal) {
    double dist_to_goal = usv.position.distanceTo(goal);
    Vector2D direction = (goal - usv.position).normalized();
    double magnitude = 0.0;

    if (dist_to_goal <= m_params.d_g) {
        magnitude = m_params.k_att * dist_to_goal;
    } else {
        magnitude = m_params.k_att * m_params.d_g;
    }
    return direction * magnitude;
}

Vector2D PAPF_Planner::calculateRepulsiveForce(const USV& usv, const std::vector<Obstacle>& obstacles, const Vector2D& goal) {
    Vector2D total_repulsive_force;
    
    for (const auto& obs : obstacles) {
        double dist_to_obs_edge = usv.position.distanceTo(obs.center) - obs.radius;
        if (dist_to_obs_edge <= 0) dist_to_obs_edge = 0.01; // Avoid division by zero
        
        if (dist_to_obs_edge < m_params.d_o) {
            double dist_to_goal = usv.position.distanceTo(goal);
            double f_rep_1_mag = m_params.k_rep * (1.0/dist_to_obs_edge - 1.0/m_params.d_o) * std::pow(dist_to_goal, m_params.n) / std::pow(dist_to_obs_edge, 2);
            double f_rep_2_mag = m_params.n * m_params.k_rep * std::pow(1.0/dist_to_obs_edge - 1.0/m_params.d_o, 2) * std::pow(dist_to_goal, m_params.n - 1) / 2.0;
            Vector2D dir1 = (usv.position - obs.center).normalized();
            Vector2D dir2 = (usv.position-goal).normalized();

            Vector2D force1 = dir1 * f_rep_1_mag;
            Vector2D force2 = dir2 * f_rep_2_mag;
            
            total_repulsive_force = total_repulsive_force + force1 + force2;
        }
    }
    return total_repulsive_force;
}

// Vector2D PAPF_Planner::calculateRepulsiveForce(const USV& usv, const std::vector<Obstacle>& obstacles, const Vector2D& goal) {
//     Vector2D total_repulsive_force;
    
//     for (const auto& obs : obstacles) {
//         double dist_to_obs_edge = usv.position.distanceTo(obs.center) - obs.radius;
//         if (dist_to_obs_edge <= 0) dist_to_obs_edge = 0.01; // Safety check
        
//         if (dist_to_obs_edge < m_params.d_o) {
//             // --- USE THE CLASSIC, MORE STABLE FORMULA ---
//             // This force only depends on distance to the obstacle, not the goal.
//             double magnitude = m_params.k_rep * (1.0 / dist_to_obs_edge - 1.0 / m_params.d_o) * (1.0 / std::pow(dist_to_obs_edge, 2));

//             Vector2D direction = (usv.position - obs.center).normalized(); // Push away from obstacle
            
//             total_repulsive_force = total_repulsive_force + direction * magnitude;
//         }
//     }
//     return total_repulsive_force;
// }


Vector2D PAPF_Planner::calculatePredictiveForce(const USV& usv, const std::vector<Obstacle>& obstacles) {
    Vector2D total_predictive_force;
    
    for (const auto& obs : obstacles) {
        double dist_to_obs_center = usv.position.distanceTo(obs.center);
        if (dist_to_obs_center <= obs.radius) continue; 
        
        double dist_to_obs_edge = dist_to_obs_center - obs.radius;

        if (dist_to_obs_edge < m_params.d_prd) {
            double angle_to_center = atan2(obs.center.y - usv.position.y, obs.center.x - usv.position.x);
            // double angle_offset = asin(obs.radius / dist_to_obs_center);
            
            double theta_tb = normalizeAngle(angle_to_center);
            double delta_theta_v = normalizeAngle(theta_tb - usv.yaw);

            double exp_common_term = exp(-dist_to_obs_edge / m_params.d_prd - std::abs(delta_theta_v));

            double f_prd_1_mag = m_params.k_prd * exp_common_term / m_params.d_prd;
            double f_prd_2_mag = m_params.k_prd * exp_common_term;

            Vector2D dir1 = (usv.position - obs.center).normalized();
            double dir2_angle = (delta_theta_v >= 0) ? theta_tb - M_PI/2.0 : theta_tb + M_PI/2.0;
            Vector2D dir2(cos(dir2_angle), sin(dir2_angle));

            Vector2D force1 = dir1 * f_prd_1_mag;
            Vector2D force2 = dir2 * f_prd_2_mag;

            total_predictive_force = total_predictive_force + force1 + force2;
        }
    }
    return total_predictive_force;
}

void PAPF_Planner::computeStep(USV& usv, const Vector2D& goal, const std::vector<Obstacle>& obstacles) {
    Vector2D f_att = calculateAttractiveForce(usv, goal);
    Vector2D f_rep = calculateRepulsiveForce(usv, obstacles, goal);
    Vector2D f_prd = calculatePredictiveForce(usv, obstacles);
    Vector2D total_force = f_att + f_rep + f_prd;

    double ideal_yaw = atan2(total_force.y, total_force.x);
    double delta_theta_ideal = normalizeAngle(ideal_yaw - usv.yaw);
    
    //  Velocity Adjustment - taken from ref. python script 
    double turning_angle_deg = delta_theta_ideal * 180.0 / M_PI;
    if (std::abs(turning_angle_deg) < m_params.theta1_deg) {
        usv.velocity = 1.7 * m_params.v_c;
    } else if (std::abs(turning_angle_deg) > m_params.theta2_deg) {
        usv.velocity = 0.7 * m_params.v_c;
    } else {
        usv.velocity = m_params.v_c;
    }

    // Angle Limit  
    double max_turn_rad = m_params.max_turning_angle_deg * M_PI / 180.0;
    double delta_theta_actual = std::clamp(delta_theta_ideal, -max_turn_rad, max_turn_rad);
    usv.yaw = normalizeAngle(usv.yaw + delta_theta_actual);

    Vector2D heading_vector(cos(usv.yaw), sin(usv.yaw));
    usv.position = usv.position + heading_vector * usv.velocity * m_params.dt;
}