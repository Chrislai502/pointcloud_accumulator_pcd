def read_pcd_header(file_path):
    header = []
    with open(file_path, 'r') as file:
        for line in file:
            header.append(line.strip())
            if line.startswith('DATA'):
                break
    return header

def compare_pcd_headers(file_path1, file_path2):
    header1 = read_pcd_header(file_path1)
    header2 = read_pcd_header(file_path2)

    print("FIRST: \n", header1)

    print("Second: \n", header2)
    
    # Compare headers
    differences = []
    for line in set(header1).union(set(header2)):
        if line not in header1:
            differences.append(f"Only in {file_path2}: {line}")
        elif line not in header2:
            differences.append(f"Only in {file_path1}: {line}")
    
    return differences

# Example usage
# file_path1 = 'single_scene_calibration/0.pcd'
file_path1 = '/home/chris/testing/lidar_cam_calib/scene_based/tools_ws/src/pointcloud_accumulator/output/merged_cloud_8.pcd'
# file_path2 = 'scene_based_mlcc/bridge/merged_cloud_8.pcd'
file_path2 = None

if file_path1 and file_path2:
    differences = compare_pcd_headers(file_path1, file_path2)
else:
    header1 = read_pcd_header(file_path1)
    print("Header: \n", )
    for i in header1:
        print(i)
if differences:
    for diff in differences:
        print(diff)
else:
    print("The headers are identical.")
