function [theta_dot_turret, theta_ddot_turret, i_dot] = turret_plant(t, V_m, theta_turret, theta_dot_turret, i) %#ok<INUSL>
%TURRET_PLANT  Elevation-drive dynamics derived from first principles.
%
%   Electrical (armature, KVL):   Lm di/dt = V_m - Rm i - Ke N theta_dot
%   Mechanical (turret side):     (Jt + N^2 Jm) theta_ddot = N Kt i - b theta_dot
%
%   Assumptions: rigid, lossless, backlash-free 100:1 gear; viscous friction only;
%   turret statically balanced about the trunnion (no gravity torque), so the
%   model is linear and is its own small-perturbation model at any elevation.
%
%   Gp(s) = theta/V = N Kt / ( s [ (Lm s + Rm)(J s + b) + N^2 Kt Ke ] )
persistent P
if isempty(P), P = guardian_params(); end
theta_ddot_turret = (P.N*P.Kt*i - P.b*theta_dot_turret)/P.J;
i_dot             = (V_m - P.Rm*i - P.Ke*P.N*theta_dot_turret)/P.Lm;
% theta_dot_turret is passed through unchanged (state derivative of theta).
end
