module OptiTrack

using Sockets: UDPSocket, bind, join_multicast_group, IPv4, recv

mutable struct OptiTrackConnection
  socket::UDPSocket

  function OptiTrackConnection(
    multicast_group="224.0.0.1",
    port=1511,
    local_interface=IPv4("0.0.0.0")
  )
    socket = UDPSocket()
    local_port = 1511
    bind(socket, local_interface, local_port)
    join_multicast_group(socket, multicast_group)
    new(socket)
  end
end

function isopen(connection::OptiTrackConnection)
  Base.isopen(connection.socket)
end

function get_rigid_body_with_id(id, mocap_frame)
  rigid_bodies = mocap_frame.rigid_bodies
  index = findfirst(rigid_bodies) do body
    body.id == id
  end
  rigid_bodies[index]
end

function next(connection::OptiTrackConnection)
  package = recv(connection.socket)
  parse_package(package)
end

function parse_vector3(buffer)
  x = read(buffer, Float32)
  y = read(buffer, Float32)
  z = read(buffer, Float32)
  (; x, y, z)
end

function parse_quaternion(buffer)
  x = read(buffer, Float32)
  y = read(buffer, Float32)
  z = read(buffer, Float32)
  w = read(buffer, Float32)
  (; x, y, z, w)
end

function parse_marker_set(buffer)
  name = String(readuntil(buffer, 0x00))
  marker_count = read(buffer, UInt32)
  markers = [parse_vector3(buffer) for _ in 1:marker_count]
  (; name, marker_count, markers)
end

function parse_rigid_body(buffer)
  id = read(buffer, UInt32)
  position = parse_vector3(buffer)
  orientation = parse_quaternion(buffer)
  marker_count = read(buffer, UInt32)
  marker_positions = [parse_vector3(buffer) for _ in 1:marker_count]
  marker_ids = [read(buffer, UInt32) for _ in 1:marker_count]
  marker_sizes = [read(buffer, Float32) for _ in 1:marker_count]
  mean_error = read(buffer, Float32)
  params = read(buffer, UInt16)
  (; id, position, orientation, marker_count, marker_positions, marker_ids, marker_sizes, mean_error, params)
end

function parse_package(package)
  buffer = IOBuffer(package)
  message_id = read(buffer, UInt16)
  payload_length = read(buffer, UInt16)
  actual_payload_length = length(package) - 4
  @assert actual_payload_length == payload_length "Message says payload has length '$(payload_length)' but payload has length '$(actual_payload_length)'"
  if message_id != 7
    @warn "Unknown message_id '$(message_id)'"
    return nothing
  end
  frame_number = read(buffer, UInt32)
  marker_set_count = read(buffer, UInt32)
  marker_sets = [parse_marker_set(buffer) for _ in 1:marker_set_count]
  unlabelled_markers_count = read(buffer, UInt32)
  # skipping the unlabelled markers
  for _ in 1:unlabelled_markers_count
    parse_vector3(buffer)
  end
  rigid_body_count = read(buffer, UInt32)
  rigid_bodies = [parse_rigid_body(buffer) for _ in 1:rigid_body_count]

  # TODO: read and parse the skeletons
  # TODO: ... and parse even more stuff from the message
  (; frame_number, marker_sets, rigid_bodies)
end

end # module
