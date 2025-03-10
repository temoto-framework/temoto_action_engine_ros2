# If invoking a graph by its name leave 'umrf_graph_json' field empty
string umrf_graph_name

# If invoking a graph directly by sending the graph JSON, leave 'umrf_graph_name' field empty
string umrf_graph_json

# REQUIRED: Name of the actor or actors that this message is meant for
string[] targets

bool name_match_required
