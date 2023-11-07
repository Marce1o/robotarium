function slBusOut = Transform(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.
    currentlength = length(slBusOut.Translation);
    for iter=1:currentlength
        slBusOut.Translation(iter) = bus_conv_fcns.ros.msgToBus.geometry_msgs.Vector3(msgIn.Translation(iter),slBusOut(1).Translation(iter),varargin{:});
    end
    slBusOut.Translation = bus_conv_fcns.ros.msgToBus.geometry_msgs.Vector3(msgIn.Translation,slBusOut(1).Translation,varargin{:});
    currentlength = length(slBusOut.Rotation);
    for iter=1:currentlength
        slBusOut.Rotation(iter) = bus_conv_fcns.ros.msgToBus.geometry_msgs.Quaternion(msgIn.Rotation(iter),slBusOut(1).Rotation(iter),varargin{:});
    end
    slBusOut.Rotation = bus_conv_fcns.ros.msgToBus.geometry_msgs.Quaternion(msgIn.Rotation,slBusOut(1).Rotation,varargin{:});
end
