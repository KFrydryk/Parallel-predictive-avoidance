stopflag = 0;
for i = 50.1 : -0.2 : 0.5
    display("in")
    for j = 50.1 : -0.2 : 0.1
        if j<5
            display("dumps/dump" + int2str(i*10) + "k" + int2str(j*10) + ".json")
            stopflag = 1
            break
        end
    end
    if stopflag == 1
        break
    end
end