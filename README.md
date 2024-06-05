# LoVCS-3D-object-recognition
In this project, we provide the Matlab code of a 3D object recognition method based on the LoVCS descriptor. The method first uses the 3D Harris+ keypoint detector to extract keypoints. Then, the LoVCS descriptor is calculated for each keypoint. By comparing the similarity of the descriptors, the correspondences are established. Finally, the 1-point traversal algorithm is used to generate the transformation hypothesis. 

Everyone is welcome to use the code for research work, but not for commerce. If you use the code, please cite my paper (Wuyong Tao, Xianghong Hua, Bufan Zhao, Dong Chen, Chong Wu, Danhua Min, LoVCS: 3D object recognition by a local voxel center based descriptor.)
In this project, four files are provided. The “ThreeDHarris_keypointtp” file is used to extracted the keypoints. The “LRF_LVCS” file is used to calculate the LRF of the descriptor. The “LVC” is applied to calculate the LoVCS descriptor. The “LoVCS 3D object recognition” file performs the 3D object recognition based on the LoVCS descriptor and 1-point traversal algorithm.

Before you carry out our algorithm, you need to calculate the point cloud resolution (pr).
