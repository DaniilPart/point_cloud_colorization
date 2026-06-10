This guide ensures all dependencies are met for an Ubuntu 24.04 system, translates the data, builds the site, and zips it up for distribution.

### Phase 1: System Dependencies

Before running the pipeline, ensure the translation and web-generation tools are installed. Because PDAL was removed from the default Ubuntu 24.04 repositories, it must be pulled from the UbuntuGIS PPA.

```bash
# 1. Install PDAL for format translation
sudo add-apt-repository ppa:ubuntugis/ubuntugis-unstable -y
sudo apt update
sudo apt install pdal -y

# 2. Download and install PotreeConverter 2.1.1 (if not already present)
wget https://github.com/potree/PotreeConverter/releases/download/2.1.1/PotreeConverter_2.1.1_x64_linux.zip
unzip PotreeConverter_2.1.1_x64_linux.zip -d ~/potree_converter
chmod +x ~/potree_converter/PotreeConverter_linux_x64/PotreeConverter
sudo cp ~/potree_converter/PotreeConverter_linux_x64/liblaszip.so /usr/local/lib/
sudo ldconfig

```

---

### Phase 2: The Processing Pipeline

Run these commands sequentially to generate the geometry, translate it, and build the web viewer.

```bash
# 1. Save a fresh map snapshot from the running aggregator.
# If periodic saving is disabled (map_save_interval_sec: 0.0), use the service utility.
ros2 run pointcloud_colorizer save_map /tmp/colored_cloud_map.ply

# Optional naming examples:
# ros2 run pointcloud_colorizer save_map run1
# ros2 run pointcloud_colorizer save_map /tmp/maps/

# 2. Generate the geometric surfels using your ROS 2 node
ros2 run pointcloud_colorizer surfel_generator /tmp/colored_cloud_map.ply /tmp/output_surfels.ply

# 3. Translate the PLY into the LAS format required by Potree 2.x
# (Note: This step preserves RGB but strips the custom PCA stretch vectors)
pdal translate /tmp/output_surfels.ply /tmp/output_surfels.las

# 4. Clean any old workspace and generate the Potree web viewer
rm -rf /tmp/potree_workspace
~/potree_converter/PotreeConverter_linux_x64/PotreeConverter /tmp/output_surfels.las -o /tmp/potree_workspace --generate-page index

```

---

### Phase 3: Compressing and Sharing

To pass this viewer to a colleague or client, you simply need to compress the generated workspace directory.

```bash
# Zip the entire workspace into a single archive
cd /tmp
zip -r 3d_forest_reconstruction.zip potree_workspace/

```

**Instructions for the Recipient:**
When you send `3d_forest_reconstruction.zip` to someone, provide them with these instructions to view it:

1. Unzip the folder.
2. Open a terminal inside the unzipped `potree_workspace` folder.
3. Start a local Python web server (browsers block local file loading for security reasons):
```bash
python3 -m http.server 8080

```


4. Open a web browser and navigate to `http://localhost:8080`.
5. **Crucial Visual Settings:** Tell them to open the left-hand menu, go to **Appearance**, change "Shape" to **Paraboloid**, set "Point Sizing" to **Fixed**, and enable both **High Quality** and **Eye-Dome Lighting** to see the map as a solid, blended surface.