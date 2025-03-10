# REQUIRED
string umrf_graph_name

# REQUIRED: Name of the actor or actors that this message is meant for
string[] targets

# REQUIRED: List of modifications applied to the graph
temoto_msgs/UmrfGraphModifier[] modifiers
