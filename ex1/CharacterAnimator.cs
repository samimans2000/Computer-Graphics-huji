using System;
using System.Collections;
using System.Collections.Generic;
using UnityEditor.Playables;
using UnityEngine;

public class CharacterAnimator : MonoBehaviour
{
    public TextAsset BVHFile; // The BVH file that defines the animation and skeleton
    public bool animate; // Indicates whether or not the animation should be running

    private BVHData data; // BVH data of the BVHFile will be loaded here
    private int prevFrame = 0;
    private int currFrame = 0; // Current frame of the animation
    private float startTime;
    private float currTime;

    // Start is called before the first frame update
    void Start()
    {
        BVHParser parser = new BVHParser();
        data = parser.Parse(BVHFile);
        CreateJoint(data.rootJoint, Vector3.zero);
        startTime = Time.time;
    }

    private void TestRotateTowardsVector()
    {
        Debug.Log("Now testing");
        List<Vector3> testVectors = new List<Vector3>();
        testVectors.Add(new Vector3(1, 1, 1));
        testVectors.Add(new Vector3(1, 2, 3));
        testVectors.Add(new Vector3(4, 2, 0));
        testVectors.Add(new Vector3(6, 9, 2));

        foreach (Vector3 testV in testVectors)
        {
            Vector3 testVNorm = testV.normalized;
            Debug.Log("NormtestV");
            Debug.Log(testVNorm);
            Matrix4x4 r = RotateTowardsVector(testV);
            Debug.Log("r");
            Debug.Log(r);
            Vector3 resV = r.MultiplyVector(Vector3.up);
            Debug.Log("resV");
            Debug.Log(resV);
            Debug.Log(resV == testVNorm);
            Debug.Log("\n\n");
        }
    }

    // Returns a Matrix4x4 representing a rotation aligning the up direction of an object with the given v
    Matrix4x4 RotateTowardsVector(Vector3 v)
    {
        // Your code here
        Vector3 normV = v.normalized;
        Debug.Log("normV is:");
        Debug.Log(normV);
        float thetaX = 90 - Mathf.Atan2(normV.y, normV.z) * Mathf.Rad2Deg;
        Matrix4x4 rX = MatrixUtils.RotateX(-thetaX);
        Vector3 rxV = rX * normV;
        Debug.Log("The z coordinate should be 0:");
        Debug.Log(rxV);
        float thetaZ = 90 - Mathf.Atan2(rxV.y, rxV.x) * Mathf.Rad2Deg;
        Matrix4x4 rZ = MatrixUtils.RotateZ(thetaZ);
        Vector3 rzrxV = rZ * rxV;
        Debug.Log("This vector should be (0,1,0):");
        Debug.Log(rzrxV);
        Matrix4x4 rXinv = rX.inverse;
        Matrix4x4 rZinv = rZ.inverse;
        Matrix4x4 rXIrZI = rXinv * rZinv;
        return rXIrZI;
    }

    // Creates a Cylinder GameObject between two given points in 3D space
    GameObject CreateCylinderBetweenPoints(Vector3 p1, Vector3 p2, float diameter)
    {
        // Your code here
        GameObject cylinder = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        Matrix4x4 t = MatrixUtils.Translate(p1 + (p2 - p1) / 2);
        Vector3 direction = p2 - p1;
        Matrix4x4 r = RotateTowardsVector(direction);
        float height = Vector3.Distance(p1, p2) / 2;
        Matrix4x4 s = MatrixUtils.Scale(new Vector3(diameter, height, diameter));
        MatrixUtils.ApplyTransform(cylinder, t * r * s);
        return cylinder;
    }

    // Creates a GameObject representing a given BVHJoint and recursively creates GameObjects for it's child joints
    GameObject CreateJoint(BVHJoint joint, Vector3 parentPosition)
    {
        joint.gameObject = new GameObject(joint.name);
        GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        Vector3 scalingVec = (joint.name == "Head") ? new Vector3(8, 8, 8) : new Vector3(2, 2, 2);
        Matrix4x4 s = MatrixUtils.Scale(scalingVec);
        MatrixUtils.ApplyTransform(sphere, s);
        Matrix4x4 t = MatrixUtils.Translate(parentPosition + joint.offset);
        sphere.gameObject.transform.parent = joint.gameObject.transform;
        MatrixUtils.ApplyTransform(joint.gameObject, t);

        foreach (BVHJoint childJoint in joint.children)
        {
            GameObject createdChildJoint = CreateJoint(childJoint, joint.gameObject.transform.position);
            GameObject cylinder =
                CreateCylinderBetweenPoints(sphere.transform.position, createdChildJoint.transform.position, 1);
            cylinder.transform.parent = joint.gameObject.transform;
        }

        return joint.gameObject;
    }


    // Transforms BVHJoint according to the keyframe channel data, and recursively transforms its children
    private void TransformJoint(BVHJoint joint, Matrix4x4 parentTransform, float[] keyframe)
    {
        Matrix4x4 localRMatrix = Matrix4x4.identity;
        if (!joint.isEndSite)
        {
            Matrix4x4[] rMatrices = new Matrix4x4[3];
            rMatrices[joint.rotationOrder.x] = MatrixUtils.RotateX(keyframe[joint.rotationChannels.x]);
            rMatrices[joint.rotationOrder.y] = MatrixUtils.RotateY(keyframe[joint.rotationChannels.y]);
            rMatrices[joint.rotationOrder.z] = MatrixUtils.RotateZ(keyframe[joint.rotationChannels.z]);
            localRMatrix = rMatrices[0] * rMatrices[1] * rMatrices[2];
        }

        Matrix4x4 localTMatrix = MatrixUtils.Translate(joint.offset);
        Matrix4x4 localTRS = localTMatrix * localRMatrix;

        Matrix4x4 globalTransform = parentTransform * localTRS;
        MatrixUtils.ApplyTransform(joint.gameObject, globalTransform);

        foreach (BVHJoint childJoint in joint.children)
        {
            TransformJoint(childJoint, globalTransform, keyframe);
        }
    }

    // Update is called once per frame
    void Update()
    {
        if (animate)
        {
            currTime = (Time.time - startTime);
            prevFrame = currFrame;
            currFrame = (int) (currTime / data.frameLength) % data.numFrames;
            if (prevFrame != currFrame)
            {
                float[] currKeyFrame = data.keyframes[currFrame];
                Vector3 positionVector = new Vector3();
                positionVector.x = currKeyFrame[data.rootJoint.positionChannels.x];
                positionVector.y = currKeyFrame[data.rootJoint.positionChannels.y];
                positionVector.z = currKeyFrame[data.rootJoint.positionChannels.z];
                Matrix4x4 positionRoot = MatrixUtils.Translate(positionVector);
                TransformJoint(data.rootJoint, positionRoot, currKeyFrame);
            }
        }
    }
}