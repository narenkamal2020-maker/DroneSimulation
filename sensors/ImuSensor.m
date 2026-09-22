classdef ImuSensor < handle
    % IMUSENSOR Simulates 3-axis Accelerometer and 3-axis Gyroscope with noise and bias.
    
    properties
        cfg
        accelNoise
        gyroNoise
        accelBias = [0.02; -0.01; 0.03]
        gyroBias  = [0.005; 0.002; -0.004]
    end
    
    methods
        function obj = ImuSensor(cfg)
            obj.cfg = cfg;
            obj.accelNoise = cfg.imu.accelNoise;
            obj.gyroNoise = cfg.imu.gyroNoise;
        end
        
        function [measAccel, measOmega] = read(obj, trueAccelWorld, R_world_body, trueOmegaBody)
            % Translates true inertial dynamics into noisy body-fixed sensor frame
            %
            % Specific force: f = R^T * (a_world + [0; 0; g])
            gVec = [0; 0; obj.cfg.drone.g];
            specForceBody = R_world_body' * (reshape(trueAccelWorld(1:3), [3, 1]) + gVec);
            
            accelNoiseVec = randn(3, 1) * obj.accelNoise;
            measAccel = specForceBody + obj.accelBias + accelNoiseVec;
            
            gyroNoiseVec = randn(3, 1) * obj.gyroNoise;
            measOmega = reshape(trueOmegaBody(1:3), [3, 1]) + obj.gyroBias + gyroNoiseVec;
        end
    end
end
