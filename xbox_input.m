ID = 1;
joy=vrjoystick(ID);

while 1
    [axes, buttons, ~] = read(joy)

    pause(1)

end
