classdef AltimeterSensor < handle
    % ALTIMETERSENSOR Simulates high-precision laser/barometric altitude measurement.
    
    properties
        cfg
        noiseSigma
    end
    
    methods
        function obj = AltimeterSensor(cfg)
            obj.cfg = cfg;
            obj.noiseSigma = cfg.altimeter.noiseSigma;
        end
        
        function measuredAlt = read(obj, trueZ)
            % Returns simulated noisy altitude AGL (above ground level)
            measuredAlt = max(0, trueZ + randn() * obj.noiseSigma);
        end
    end
end
