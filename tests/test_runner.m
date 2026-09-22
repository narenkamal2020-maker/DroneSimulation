% TEST_RUNNER Runner script that executes runTests and records output via diary.
try
    diary('test_output.txt');
    diary on;
    addpath(genpath('.'));
    res = runTests();
    diary off;
    if res.success
        exit(0);
    else
        exit(1);
    end
catch ME
    fprintf('FATAL ERROR DURING TEST EXECUTION: %s\n', ME.message);
    if ~isempty(ME.stack)
        for s = 1:length(ME.stack)
            fprintf('  in %s at line %d\n', ME.stack(s).name, ME.stack(s).line);
        end
    end
    diary off;
    exit(2);
end
