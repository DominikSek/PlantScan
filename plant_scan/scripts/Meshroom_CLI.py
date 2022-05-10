#!/usr/bin/env python3
import sys
import os, os.path
import math
import time
from pathlib import Path

dirname = os.path.dirname(os.path.abspath(__file__))  # Absolute path of this file

verboseLevel = "\"" + "error" + "\""  # detail of the logs (error, info, etc)


def SilentMkdir(theDir):    # function to create a directory
    try:
        os.mkdir(theDir)
    except:
        pass
    return 0


def cameraInit(binPath,baseDir,imgDir):

    taskFolder = "/1_CameraInit"
    SilentMkdir(baseDir + taskFolder)
    imageFolder = "\"" + imgDir + "\""
    sensorDatabase = "\"" + str(Path(binPath).parent) + "/share/aliceVision/cameraSensors.db" "\""
   
    output = "\"" + baseDir + taskFolder + "/cameraInit.sfm" + "\""

    cmdLine = binPath + "aliceVision_cameraInit" + f" --imageFolder {imageFolder} --sensorDatabase {sensorDatabase} --output {output}"
    cmdLine += " --defaultFieldOfView 45"
    #cmdLine += " --allowSingleView 1"
    cmdLine += " --verboseLevel " + verboseLevel
    os.system(cmdLine)
    return 0


def featureExtraction(binPath,baseDir , numberOfImages , imagesPerGroup=40):

    taskFolder = "/2_FeatureExtraction"
    SilentMkdir(baseDir + taskFolder)
    useCPU = 0 ### Change this if CPU is needed


    inputFolder = "\"" + baseDir + "/1_CameraInit/cameraInit.sfm" + "\""
    output = "\"" + baseDir + taskFolder + "\""
    cmdLine = binPath + "/aliceVision_featureExtraction" + f" --input {inputFolder} --output {output}"

    cmdLine += f"--forceCpuExtraction {useCPU}"     ### Here it is possibly 1, testing further

    #Sending images in groups if there is too many (removing the overcrowding)
    if numberOfImages > imagesPerGroup:
        numberOfGroups=int(math.ceil( numberOfImages/imagesPerGroup))
        for i in range(numberOfGroups):
            cmd = cmdLine + " --rangeStart {} --rangeSize {} ".format(i*imagesPerGroup,imagesPerGroup)
            os.system(cmd)
    else:
        os.system(cmdLine)


def imageMatching(binPath,baseDir):

    taskFolder = "/3_ImageMatching"
    SilentMkdir(baseDir + taskFolder)

    inputFolder = "\"" + baseDir + "/1_CameraInit/cameraInit.sfm" + "\""
    featuresFolders = "\"" + baseDir + "/2_FeatureExtraction" + "\""
    output = "\"" + baseDir + taskFolder + "/imageMatches.txt" + "\""

    cmdLine = binPath + "/aliceVision_imageMatching" + f" --input {inputFolder} --featuresFolders {featuresFolders} --output {output}"
    cmdLine += " --tree " + "\"" + str(Path(binPath).parent) + "/share/aliceVision/vlfeat_K80L3.SIFT.tree\""
    cmdLine += " --verboseLevel " + verboseLevel

    os.system(cmdLine)


def featureMatching(binPath,baseDir,numberOfImages,imagesPerGroup=40):

    taskFolder = "/4_featureMatching"
    SilentMkdir(baseDir + taskFolder)

    inputFolder = "\"" + baseDir + "/1_CameraInit/cameraInit.sfm" + "\""
    output = "\"" + baseDir + taskFolder + "\""
    featuresFolders = "\"" + baseDir + "/2_FeatureExtraction" + "\""
    imagePairsList = "\"" + baseDir + "/3_ImageMatching/imageMatches.txt" + "\""

    cmdLine = binPath + "/aliceVision_featureMatching" + f" --input {inputFolder} --featuresFolders {featuresFolders} --output {output}"
    cmdLine += f"--imagePairsList {imagePairsList}"
    cmdLine += " --knownPosesGeometricErrorMax 5"       ##This is default in meshroom, could be changed in the future
    cmdLine += " --verboseLevel " + verboseLevel

    # These are all default parameters given by meshroom, except the debug files, that we use.
    cmdLine += " --describerTypes sift --photometricMatchingMethod ANN_L2 --geometricEstimator acransac --geometricFilterType fundamental_matrix --distanceRatio 0.8"
    cmdLine += " --maxIteration 2048 --geometricError 0.0 --maxMatches 0"
    cmdLine += " --savePutativeMatches False --guidedMatching False --matchFromKnownCameraPoses False --exportDebugFiles True"

    #As above, sending in groups
    if numberOfImages > imagesPerGroup:
        numberOfGroups = math.ceil(numberOfImages / imagesPerGroup)
        for i in range(numberOfGroups):
            cmd = cmdLine + f" --rangeStart {i*imagesPerGroup} --rangeSize {imagesPerGroup} "
            os.system(cmd)
    else:
        os.system(cmdLine)


def structureFromMotion(binPath, baseDir):

    taskFolder = "/5_structureFromMotion"
    SilentMkdir(baseDir + taskFolder)

    inputFolder = "\"" + baseDir + "/1_CameraInit/cameraInit.sfm" + "\""
    output = "\"" + baseDir + taskFolder + "/sfm.abc" + "\" "
    outputViewsAndPoses = "\"" + baseDir + taskFolder + "/cameras.sfm" + "\""       # Output sfmdata file with cameras (views and poses)
    extraInfoFolder = "\"" + baseDir + taskFolder + "\""
    featuresFolders = "\"" + baseDir + "/2_FeatureExtraction" + "\""
    matchesFolders = "\"" + baseDir + "/4_featureMatching" + "\""

    cmdLine = binPath + "/aliceVision_incrementalSfM" + f" --input {inputFolder} --output {output}"

    # Necessary folders to export data
    # It pulls all the parameters given in featureMatching()
    cmdLine += f"--outputViewsAndPoses {outputViewsAndPoses} --extraInfoFolder {extraInfoFolder} --featuresFolders {featuresFolders} --matchesFolders {matchesFolders}"
    cmdLine += " --verboseLevel " + verboseLevel

    os.system(cmdLine)


def prepareDenseScene(binPath, baseDir):
    taskFolder = "/6_PrepareDenseScene"
    SilentMkdir(baseDir + taskFolder)

    inputFolder = "\"" +  baseDir + "/5_structureFromMotion/sfm.abc" + "\""
    output = "\"" + baseDir + taskFolder + "\" "

    cmdLine = binPath + "/aliceVision_prepareDenseScene" + f" --input {inputFolder}  --output {output} "
    cmdLine += " --verboseLevel " + verboseLevel
    os.system(cmdLine)


def depthMap(binPath, baseDir, numberOfImages, groupSize=6, downscale=2):
    taskFolder = "/7_DepthMap"
    SilentMkdir(baseDir + taskFolder)

    inputFolder = "\"" + baseDir + "/5_structureFromMotion/sfm.abc" + "\""
    output = "\"" + baseDir + taskFolder + "\""
    imagesFolder = "\"" + baseDir + "/6_PrepareDenseScene" + "\""

    cmdLine = binPath + "/aliceVision_depthMapEstimation" + f" --input {inputFolder}  --output {output} --imagesFolder {imagesFolder}"
    cmdLine += " --verboseLevel " + verboseLevel

    # Downscale = 2 is the default value given by meshroom
    cmdLine += " --downscale " + str(downscale)

    # Splitting groups again (found on github)
    for i in range(int(math.ceil(numberOfImages / groupSize))):
        groupStart = groupSize * i
        if groupSize > 1:
            cmd = cmdLine + f" --rangeStart {str(groupStart)} --rangeSize {groupSize}"
            os.system(cmd)


def depthMapFilter(binPath,baseDir):

    taskFolder = "/8_DepthMapFilter"
    SilentMkdir(baseDir + taskFolder)

    inputFolder = "\"" + baseDir + "/5_structureFromMotion/sfm.abc" + "\""
    output = "\"" + baseDir + taskFolder + "\""
    depthMapsFolder = "\"" + baseDir + "/7_DepthMap" + "\""

    cmdLine = binPath + "/aliceVision_depthMapFiltering" + f" --input {inputFolder}  --output {output} --depthMapsFolder {depthMapsFolder}"
    cmdLine += " --verboseLevel " + verboseLevel

    os.system(cmdLine)


def meshing(binPath, baseDir, maxInputPoints = 50000000, maxPoints=5000000):
    taskFolder = "/9_Meshing"
    SilentMkdir(baseDir + taskFolder)

    inputFolder = "\"" + baseDir + "/5_structureFromMotion/sfm.abc" + "\""
    output = "\"" + baseDir + taskFolder + "/densePointCloud.abc" "\""
    outputMesh = "\""  + baseDir + taskFolder + "/mesh.obj" + "\""
    depthMapsFilterFolder = "\"" + baseDir + "/8_DepthMapFilter" + "\""

    cmdLine = binPath + "/aliceVision_meshing" + f" --input {inputFolder}  --output {output} --outputMesh {outputMesh} --depthMapsFolder {depthMapsFilterFolder}"

    cmdLine += " --maxInputPoints " + str(maxInputPoints)
    cmdLine += " --maxPoints " + str(maxPoints)
    cmdLine += " --verboseLevel " + verboseLevel

    os.system(cmdLine)


def meshFiltering(binPath, baseDir, keepLargestMeshOnly="True"):
    taskFolder = "/10_MeshFiltering"
    SilentMkdir(baseDir + taskFolder)

    inputMesh = "\"" + baseDir + "/9_Meshing/mesh.obj" + "\""
    outputMesh = "\"" + baseDir + taskFolder + "/mesh.obj" + "\""

    cmdLine = binPath + "/aliceVision_meshFiltering" + f" --inputMesh {inputMesh}  --outputMesh {outputMesh}"
    cmdLine += " --verboseLevel " + verboseLevel
    cmdLine += " --keepLargestMeshOnly " + keepLargestMeshOnly

    os.system(cmdLine)


def meshDecimate(binPath,baseDir , simplificationFactor=0.8 , maxVertices=15000):
    taskFolder = "/11_MeshDecimate"
    SilentMkdir(baseDir + taskFolder)

    inputMesh = "\""  + baseDir + "/10_MeshFiltering/mesh.obj" + "\""
    outputMesh = "\""  + baseDir + taskFolder + "/mesh.obj" + "\""

    cmdLine = binPath + "/aliceVision_meshDecimate" + f" --input {inputMesh}  --output {outputMesh}"

    cmdLine += " --verboseLevel " + verboseLevel
    cmdLine += " --simplificationFactor " + str(simplificationFactor)
    #cmdLine += " --maxVertices " + str(maxVertices)

    os.system(cmdLine)


def meshResampling(binPath,baseDir , simplificationFactor=0.8 , maxVertices=15000):
    taskFolder = "/12_MeshResampling"
    SilentMkdir(baseDir + taskFolder)

    inputMesh = "\"" + baseDir +  "/11_MeshDecimate/mesh.obj" + "\""
    outputMesh = "\"" + baseDir  + taskFolder + "/mesh.obj" + "\""

    cmdLine = binPath + "/aliceVision_meshResampling" + f" --input {inputMesh}  --output {outputMesh}"
    cmdLine += " --verboseLevel " + verboseLevel
    cmdLine += " --simplificationFactor " + str(simplificationFactor)
    #cmdLine += " --maxVertices " + str(maxVertices)

    os.system(cmdLine)


def texturing(binPath, baseDir, finish_Dir, textureSide = 4096 , downscale=4, unwrapMethod = "Basic", meshDecAndResample=True):
    taskFolder = "/Finished_texture"
    SilentMkdir(finish_Dir + taskFolder)

    inputFolder = "\"" + baseDir + "/9_Meshing/densePointCloud.abc" + "\""
    imagesFolder = "\"" + baseDir + "/6_PrepareDenseScene" "\""
    if meshDecAndResample:
        inputMesh = "\"" + baseDir + "/12_MeshResampling/mesh.obj" + "\""
    else:
        inputMesh = "\"" + baseDir + "/10_MeshFiltering/mesh.obj" + "\""
    output = "\"" + finish_Dir + taskFolder + "\""

    cmdLine = binPath + "/aliceVision_texturing" + f" --input {inputFolder} --inputMesh {inputMesh} --output {output} --imagesFolder {imagesFolder}"
    cmdLine += " --textureSide " + str(textureSide)
    cmdLine += " --downscale " + str(downscale)
    cmdLine += " --verboseLevel " + verboseLevel
    cmdLine += " --unwrapMethod " + unwrapMethod

    os.system(cmdLine)


def count_images(imgDir):
    return len([name for name in os.listdir(imgDir) if os.path.isfile(os.path.join(imgDir, name))])   ## number of files in the folder

def __run__(binPath,baseDir,imgDir, finish_Dir):
    numberOfImages = count_images(imgDir)
    SilentMkdir(baseDir)

    cameraInit(binPath, baseDir, imgDir)
    featureExtraction(binPath, baseDir, numberOfImages)
    imageMatching(binPath, baseDir)
    featureMatching(binPath, baseDir, numberOfImages)
    structureFromMotion(binPath, baseDir)
    prepareDenseScene(binPath, baseDir)
    depthMap(binPath, baseDir, numberOfImages)
    depthMapFilter(binPath, baseDir)
    meshing(binPath, baseDir)
    meshFiltering(binPath, baseDir)
    meshDecimate(binPath, baseDir, simplificationFactor=0.5)
    meshResampling(binPath, baseDir, simplificationFactor=0.5)
    texturing(binPath, baseDir, finish_Dir, downscale=2, textureSide=8192)

def main():

    # Pass the arguments of the function as parameters in the command line code
    binPath = sys.argv[1]           ##  --> path of the binary files from Meshroom
    baseDir = sys.argv[2]           ##  --> name of the Folder containing the process (a new folder will be created), metadata
    imgDir = sys.argv[3]            ##  --> Folder containing the images
    if len(sys.argv) == 5:
        finish_Dir = sys.argv[4]        ##  --> Folder with finished texture
    else:
        finish_Dir = baseDir
    
    __run__(binPath, baseDir, imgDir, finish_Dir)
    print("-------------------- DONE --------------------")

    #raw_input("press any key to close")


main()
