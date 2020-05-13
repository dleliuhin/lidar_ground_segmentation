# lidar_ground_segmentation

Lidar data filtering method based on Cloth Simulation. This is the code for the article:

W. Zhang, J. Qi*, P. Wan, H. Wang, D. Xie, X. Wang, and G. Yan, “An Easy-to-Use Airborne LiDAR Data Filtering Method Based on Cloth Simulation,” Remote Sens., vol. 8, no. 6, p. 501, 2016. http://www.mdpi.com/2072-4292/8/6/501/htm

**lidar_ground_segmentation** is developed based on Qt C++. With **lidar_ground_segmentation**, users can receive point cloud data stored at Json file, visualize and segment them.<br />

---

## Installation

The installation procedures in Linux Ubuntu 16.04/14.04 32-bit LTS or Linux Mint 19.* 64-bit are shown here as examples.<br />

---

### Dependencies

#### Qt:

Service requires Qt 5 [Qt 5.0.0+](https://www.qt.io/download-open-source#section-2). You need to visit Qt downloads page an download a 32-bit or 64-bit Linux installation depending your version of Ubuntu. The installation file can be also downloaded latest version through the command line using wget. Visit Qt install wiki for more info [Qt install wiki](https://wiki.qt.io/Install_Qt_5_on_Ubuntu).<br />

For data visualization go to [ibeo_cluster.pro](http://http://bb.niias/projects/ADO/repos/livox_cluster/browse/livox_cluster.pro) and change the flag from:<br />
```
DEFINES -= GUI
```
to<br />
```
DEFINES += GUI
```
and rebuild the project.<br />

***Warning! Data visualization mode requires Qt Creator version 5.10.0 or more.***<br />

For service state trace change flag from:<br />
```
DEFINES -= TRACE
```
to<br />
```
DEFINES += TRACE
```

---

#### Testing

Google Tests used for Unit-testing.<br />
For installation it is necessary to run:<br />
```
sudo apt install libgtest-dev
cd /usr/src/gtest/
sudo cmake -DBUILD_SHARED_LIBS=ON
sudo make
sudo cp *.so /usr/lib
```

Start unit tests:<br />
```
cd lidar_ground_segmentation
./scripts/test.sh
```

---

## Development setup

Describing how to install all development dependencies and how to run an automated test-suite of some kind. Potentially do this for multiple platforms.<br />

```
cd lidar_ground_segmentation
git checkout release
./scripts/build.sh
./scripts/run.sh
```

Build project without updating submodules:<br />

```
cd lidar_ground_segmentation
git checkout release
./scripts/build.sh
```

Or using Qt Creator:<br />

*Projects->Run->* and insert the code:<br />

---

## [Release History](./HISTORY.md)

---
    
## Contributing

1. Clone it (<ssh://git@github.com:dleliuhin/lidar_ground_segmentation.git>)
2. Create your feature branch (`git checkout -b feature/fooBar`)
3. Commit your changes (`git add . & git commit -m "Feature. Add some fooBar."`)
4. Push to the branch (`git push origin feature/fooBar`)
5. Create a new Pull Request to `develop`

---

## Support

Reach out to me at one of the following places!

- Telegram at <a href="http://https://telegram.org" target="_blank">`@DLeliuhin`</a>
- Email at dleliuhin@gmail.com
---

## FAQ

---
