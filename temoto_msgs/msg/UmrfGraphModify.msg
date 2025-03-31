# REQUIRED
string umrf_graph_name

# REQUIRED: Name of the actor or actors that this message is meant for
string[] targets

# REQUIRED: Modified version of the graph as a JSON string
string modified_graph

# OPTIONAL & EXPERIMENTAL: Full name of the action (name + "_" + id) where
# the modified graph should continue its execution
string continue_from
