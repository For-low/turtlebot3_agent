# Coordinates are in the 'map' frame
# Replace with your actual coordinates from RViz/Gazebo
locations = {
    "origin":           (0.0, 0.0),
    "point_a":          (4.277, 1.045),
    "point_b":          (4.258, -0.050),
    "point_c":          (1.132, 2.636),
    "point_d":          (-0.566, 0.484),
    "point_e":          (1.494, -0.070),
    "point_f":          (3.877, 1.839),
    # Add more meaningful names based on your map
    "charging_station": (-0.566, 0.484), # Example: map point_d to charging_station
    "kitchen":          (4.277, 1.045), # Example: map point_a to kitchen
    "office":           (1.132, 2.636), # Example: map point_c to office
    "hallway_corner":   (4.258, -0.050), # Example: map point_b
    "desk":             (1.494, -0.070), # Example: map point_e
    "meeting_point":    (3.877, 1.839), # Example: map point_f
}

def get_location_coords(location_name: str):
    """Gets coordinates for a named location (case-insensitive, ignores spaces)."""
    normalized_name = location_name.lower().replace(" ", "_")
    return locations.get(normalized_name)

def get_all_location_names():
    """Returns a list of all defined location names."""
    return sorted(list(locations.keys())) # Return sorted list

def get_closest_location_name(partial_name: str):
    """Finds the best matching location name."""
    if not partial_name:
        return None
    normalized_query = partial_name.lower().replace(" ", "_")

    # Exact match first
    if normalized_query in locations:
        return normalized_query

    # Simple substring match (can be improved with fuzzy matching if needed)
    matches = [loc for loc in locations.keys() if normalized_query in loc]
    if len(matches) == 1:
         return matches[0]
    elif len(matches) > 1:
         # Ambiguous, return None or a special indicator? For now, None.
         print(f"Ambiguous location name '{partial_name}'. Matches: {matches}")
         return None

    return None # No match found