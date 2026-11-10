using UnityEngine;

// Attach to an empty GameObject in an empty 3D scene. No model needed.
public class GloveDemo : MonoBehaviour
{
    void Awake()
    {
        GloveReceiver receiver=GetComponent<GloveReceiver>();
        if (receiver==null) receiver=gameObject.AddComponent<GloveReceiver>();
        GameObject root=new GameObject("Generated glove hand");
        root.transform.SetParent(transform,false);
        Box(root.transform,new Vector3(0,0,0),new Vector3(3.7f,0.6f,3.7f));
        GloveHand hand=root.AddComponent<GloveHand>();
        hand.receiver=receiver;
        hand.fingers=new GloveHand.Finger[5];
        string[] names={"Thumb","Index","Middle","Ring","Little"};
        float[] x={-1.8f,-1.3f,-0.43f,0.43f,1.3f};
        float[] length={0.85f,1.15f,1.3f,1.18f,0.95f};
        for (int i=0;i<5;i++) {
            GloveHand.Finger f=new GloveHand.Finger();
            f.name=names[i];
            if (i==0) f.maxAngles=new float[]{35,50,40};
            Transform parent=root.transform;
            for (int j=0;j<3;j++) {
                Transform joint=new GameObject(names[i]+" joint "+j).transform;
                joint.SetParent(parent,false);
                joint.localPosition=j==0 ? new Vector3(x[i],0,i==0?-0.5f:1.8f) : new Vector3(0,0,length[i]);
                if (i==0 && j==0) joint.localRotation=Quaternion.Euler(0,-50,0);
                Box(joint,new Vector3(0,0,length[i]*0.5f),new Vector3(0.55f,0.55f,length[i]*0.94f));
                f.joints[j]=joint;
                parent=joint;
            }
            hand.fingers[i]=f;
        }
        IndexContactSimulation simulation = root.AddComponent<IndexContactSimulation>();
        simulation.hand = hand;
        hand.contactSimulation = simulation;
        receiver.contactSimulation = simulation;
        Camera camera=Camera.main;
        if (camera==null) {
            GameObject cameraObject=new GameObject("Glove camera");
            camera=cameraObject.AddComponent<Camera>();
            cameraObject.tag="MainCamera";
        }
        camera.transform.position=transform.TransformPoint(new Vector3(8,11,12));
        camera.transform.LookAt(transform.TransformPoint(new Vector3(0,0,1.6f)));
        camera.nearClipPlane=0.1f;
        camera.farClipPlane=1000;
        if (FindObjectOfType<Light>()==null) {
            Light light=new GameObject("Glove light").AddComponent<Light>();
            light.type=LightType.Directional;
            light.transform.rotation=Quaternion.Euler(45,-30,0);
        }
    }
    static Material skin;
    static void Box(Transform parent,Vector3 position,Vector3 scale)
    {
        GameObject box=GameObject.CreatePrimitive(PrimitiveType.Cube);
        box.transform.SetParent(parent,false);
        box.transform.localPosition=position;
        box.transform.localScale=scale;
        Destroy(box.GetComponent<Collider>());
        // The primitive's default material uses Standard, which a build strips (nothing in the
        // scene references it) -> pink, unlit. Legacy Diffuse is in Always Included Shaders:
        // lit, casts and receives shadows.
        if (skin==null) {
            Shader shader=Shader.Find("Legacy Shaders/Diffuse");
            if (shader!=null) skin=new Material(shader){color=new Color(0.93f,0.76f,0.64f)};
        }
        if (skin!=null) box.GetComponent<Renderer>().sharedMaterial=skin;
    }
}
