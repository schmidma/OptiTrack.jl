# OptiTrack.jl
Receive NatNet messages from OptiTrack motion capture system

## Quickstart Guide

```julia
using OptiTrack: OptiTrackConnection, receive, get_rigid_body_with_id

# create an UDP socket that listens for OptiTrack messages
connection = OptiTrackConnection()
# read a single message from the socket
mocap_frame = receive(connection)
# use the helper function to extract the body information for a given body id
get_rigid_body_with_id(body_id, mocap_frame)
```
