# Snapshot of every vessel the renderer should currently display.
# Each message replaces the previous one entirely: vessels missing from it
# should be removed, new ones spawned.
std_msgs/Header header
RendererVessel[] vessels
