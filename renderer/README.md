# Parkme Renderer

This program takes input from the parkme unilidar [https://github.com/cueltschey/parkme-unilidar](https://github.com/cueltschey/parkme-unilidar) as input from a websocket. It then displays this data in realtime, and colors pointcloud data based on euclidean clustering. These clusters are sent to the parkme-ai component, which classifies the object. The classification is relayed back, and displayed in the browser.
