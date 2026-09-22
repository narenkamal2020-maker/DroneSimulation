classdef GpsSensor < handle
    % GPSSENSOR Simulates 3D GPS with Gaussian noise and slow drift bias.
    
    properties
        cfg
        noiseHoriz
        noiseVert
        drift = [0; 0; 0]
        driftRate = 0.002
    end
    
    methods
        function obj = GpsSensor(cfg)
            obj.cfg = cfg;
            obj.noiseHoriz = cfg.gps.noiseSigmaHoriz;
            obj.noiseVert = cfg.gps.noiseSigmaVert;
        end
        
        function measuredPos = read(obj, truePos, dt)
            % Returns simulated noisy 3D GPS position
            obj.drift = obj.drift + randn(3, 1) * (obj.driftRate * sqrt(dt));
            noise = [randn() * obj.noiseHoriz; ...
                     randn() * obj.noiseHoriz; ...
                     randn() * obj.noiseVert];
            measuredPos = reshape(truePos(1:3), [3, 1]) + obj.drift + noise;
        end
    end
end
