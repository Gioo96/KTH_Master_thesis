function getMeasurements(topic, callback)

go = true;

global vel;
while go
   
    rossubscriber(topic, callback, 'DataFormat', 'struct');
    disp(vel(1))
    if vel(1) > 0

        go = false;
    end
end

end