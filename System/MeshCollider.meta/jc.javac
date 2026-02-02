/* @Author Sr Mil Games */ 
// test class, extremely poorly optimized
 public class MeshCollider extends Component {
    public Transform transform;

    transient List<Vector3> localVerts = new ArrayList<Vector3>();

    public MeshCollider() {
    }

    public void init(SpatialObject obj) {
        this.transform = obj.getTransform();
        ModelRenderer mr = obj.findComponent(ModelRenderer.class);
        Vertex v = mr.getVertex();

        // Load vertices
        NativeFloatBuffer buf = v.getVerticesBuffer();
        int vc = buf.capacity() / 3;
        for (int i = 0; i < vc; i++) {
            localVerts.add(new Vector3(
                buf.get(i * 3),
                buf.get(i * 3 + 1),
                buf.get(i * 3 + 2)
            ));
        }

     }

    public List<Vector3> getLocalVerts() {
        return localVerts;
    }
}
 