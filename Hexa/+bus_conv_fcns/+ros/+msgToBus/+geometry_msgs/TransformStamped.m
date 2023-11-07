function slBusOut = TransformStamped(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.
    currentlength = length(slBusOut.Header);
    for iter=1:currentlength
        slBusOut.Header(iter) = bus_conv_fcns.ros.msgToBus.std_msgs.Header(msgIn.Header(iter),slBusOut(1).Header(iter),varargin{:});
    end
    slBusOut.Header = bus_conv_fcns.ros.msgToBus.std_msgs.Header(msgIn.Header,slBusOut(1).Header,varargin{:});
    slBusOut.ChildFrameId_SL_Info.ReceivedLength = uint32(strlength(msgIn.ChildFrameId));
    currlen  = min(slBusOut.ChildFrameId_SL_Info.ReceivedLength, length(slBusOut.ChildFrameId));
    slBusOut.ChildFrameId_SL_Info.CurrentLength = uint32(currlen);
    slBusOut.ChildFrameId(1:currlen) = uint8(char(msgIn.ChildFrameId(1:currlen))).';
    currentlength = length(slBusOut.Transform);
    for iter=1:currentlength
        slBusOut.Transform(iter) = bus_conv_fcns.ros.msgToBus.geometry_msgs.Transform(msgIn.Transform(iter),slBusOut(1).Transform(iter),varargin{:});
    end
    slBusOut.Transform = bus_conv_fcns.ros.msgToBus.geometry_msgs.Transform(msgIn.Transform,slBusOut(1).Transform,varargin{:});
end
