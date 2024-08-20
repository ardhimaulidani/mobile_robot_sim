def angle_difference(current_angle, previous_angle):
    # Normalize both angles to the range [-180, 180)
    
    # Calculate the difference
    delta_angle = ((current_angle + 180) % 360 - 180) - ((previous_angle + 180) % 360 - 180)
    
    # Ensure the delta_angle is within [-180, 180)
    if delta_angle > 180:
        delta_angle -= 360
    elif delta_angle < -180:
        delta_angle += 360
    
    return delta_angle

# Example usage:
previous_angle = 180  # in degrees
current_angle = -179    # in degrees
delta_angle = angle_difference(current_angle, previous_angle)
print(f"Delta angle: {delta_angle} degrees")  # Output should be -2 degrees