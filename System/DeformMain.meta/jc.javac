  
  
  
/* @Author Sr Mil Games */  
  
// Code Made by VitorG and Huge Helped by Gpt  
  
import java.util.HashSet;  
import java.util.HashMap;  
import java.util.Map;  
import java.util.Set;  
  
// ================================  
// Sistema de edição e configuração visual no editor  
// ================================  
@Order(idx = {0})  
boolean Editor_Mode;  
  
@Order(idx = {1})  
boolean Editor_Clear_All;  
  
@Order(idx = {2})  
boolean Update_Mesh;  
  
@Order(idx = {3})  
private int atualizado;  
  
@Order(idx = {4})  
boolean Enable_Visible_Points;  
  
@Order(idx = {5})  
private boolean lastEVisiblePoints = false;  
  
@Order(idx = {6})  
private float[] tempVerts;  
  
@Order(idx = {7})  
float Bending_Constraints_Stiff = 0.8f;  
  
@Order(idx = {8})  
float Edge_Constraints_Stiff = 0.8f;  
  
@Order(idx = {9})  
boolean Reset_Points_Positions = false;   
// ================================  
// Stiffness e controles diversos  
// ================================  
@Order(idx = {10})  
float Stiffness=0.6f;  
  
@Order(idx = {11})  
public float Velocity_Damping = 0.985f; // 0.95..0.995 (maior = mais fluido)  
  
@Order(idx = {12})  
boolean Do_Not_Use_Phys;  
  
// ================================  
// Colisores externos (adicionados manualmente na cena)  
// ================================  
      
@Order(idx = {13})
public float pointRadius = 0.02f; // ajuste conforme escala da malha

@Order(idx = {20})  
public List<SpatialObject> Colliders_Mesh = new ArrayList<SpatialObject>();  
  
@Order(idx = {21})  
public List<SpatialObject> Colliders_Box = new ArrayList<SpatialObject>();  
  
@Order(idx = {22})  
public List<SpatialObject> Colliders_Sphere = new ArrayList<SpatialObject>();  
// ================================  
// Parâmetros de vento  
// ================================  
@Order(idx = {30})  
public Vector3 Wind_Global_Direction = new Vector3(1, 0, 0).normalize();  
  
@Order(idx = {31})  
public float Wind_Strength = 50.0f;  
  
@Order(idx = {32})  
public float Turbulence = 1.5f;  
  
@Order(idx = {33})  
public float Pressure_Coefficient = 0.15f;  
  
@Order(idx = {34})  
public float Wind_Noise_Scale = 0.1f;  
  
@Order(idx = {35})  
public float Wind_Frequency = 0.3f;  
  
@Order(idx = {36})  
float Drag_Coefficient;   
// ================================  
// Constantes / Globals  
// ================================  
@Order(idx = {40})  
public static float timeStep = 0.016f;  
  
@Order(idx = {41})  
public float Gravity = -9.8f;  
  
// ================================  
// Controle de qualidade / performance  
// ================================  
@Order(idx = {50})  
public int Quality_Level_Max2 = 0; // 0 = baixo, 1 = médio, 2 = alto  
  
@Order(idx = {51})  
public boolean Self_Collision;  
  
@Order(idx = {52})  
public boolean Advanced_Wind;  
  
@Order(idx = {53})  
public boolean Seam_Constraints;  
  
@Order(idx = {54})  
public boolean Mesh_Collision;  
  
// ================================  
// Estado interno do simulador  
// ================================  
@Order(idx = {70})  
private clothPoint[] points;  
  
@Order(idx = {71})  
private List<ClothConstraint> constraints = new ArrayList<ClothConstraint>();  
  
@Order(idx = {72})  
private SpatialHash spatialHash;  
  
@Order(idx = {73})  
private float cellSize = 0.2f;  
  
@Order(idx = {74})  
private Vertex mesh;  
  
@Order(idx = {75})  
private NativeFloatBuffer vertexBuffer;  
  
@Order(idx = {76})  
private NativeFloatBuffer normalBuffer;  
  
// ================================  
// Variáveis temporárias / contadores  
// ================================  
@Order(idx = {80})  
private Vector3 tempVec1 = new Vector3();  
  
@Order(idx = {81})  
private int frameCount = 0;  
  
// physics accumulator + damping explícito  
private float physAccumulator = 0f;  
  
@Order(idx = {82})  
private float windTimeAccumulator = 0;  
  
// temporários para vento (evita muitos new Vector3 por frame)  
private Vector3 windTemp = new Vector3();   
private Vector3 relVelTemp = new Vector3();   
  
//aplicar mesh apenas quando necessário  
private boolean meshDirty = false;   
  
// ================================  
// Dados adicionais / grupos / colisores de malha  
// ================================  
@Order(idx = {90})  
private Map<String, List<Integer>> seamGroups;  
  
@Order(idx = {91})  
private List<MeshCollider> meshColliders = new ArrayList<MeshCollider>();  
  
// ================================  
// Flags de fixação de bordas / atualização de fixos  
// ================================  
@Order(idx = {100})  
public boolean Fix_Left = false;  
  
@Order(idx = {101})  
public boolean Fix_Right = false;  
  
@Order(idx = {102})  
public boolean Fix_Top = false;  
  
@Order(idx = {103})  
public boolean Fix_Bottom = false;  
  
@Order(idx = {104})  
public boolean Update_Fixed_Pos;  

//====================
@Order(idx = {110})
public int normalsUpdateInterval = 2; // recalcula normais a cada N frames (>=1)

@Order(idx = {111})
public boolean forceRecalculateNormals = false; // debug

private int framesSinceNormals = 0;
//====================

private HashMap<ClothConstraint, Float> constraintRestLengths = new HashMap<ClothConstraint, Float>();

private HashMap<String, List<Integer>> buildEditorSeamGroups() {  
  HashMap<String, List<Integer>> groups = new HashMap<String, List<Integer>>();  
  float threshold = 0.0001f; // tolerância para considerar mesma posição  
  
  int childCount = myObject.getChildCount();  
  for (int i = 0; i < childCount; i++) {  
    SpatialObject pointObj = myObject.findChildObject("ponto-" + i);  
    if (pointObj == null) continue;  
  
    Vector3 pos = pointObj.getPosition();  
    // Arredonda a posição para uma "chave" para agrupar  
    float x = Math.round(pos.getX() / threshold) * threshold;  
    float y = Math.round(pos.getY() / threshold) * threshold;  
    float z = Math.round(pos.getZ() / threshold) * threshold;  
  
    String key = String.format("%.6f_%.6f_%.6f", x, y, z);  
  
    if (!groups.containsKey(key)) {  
      groups.put(key, new ArrayList<Integer>());  
    }  
    groups.get(key).add(i);  
  }  
  return groups;  
}  

  private void sanitizeTempVerts(float[] verts, NativeFloatBuffer prevBuffer) {
  if (verts == null) return;
  int len = verts.length;
  for (int i = 0; i < len; i++) {
    float v = verts[i];
    if (Float.isNaN(v) || Float.isInfinite(v)) {
      float fallback = 0f;
      if (prevBuffer != null && i < prevBuffer.capacity()) fallback = prevBuffer.get(i);
      verts[i] = fallback;
   }
  }
}
private void updateEditorPointRenderersOptimized() {  
  HashMap<String, List<Integer>> editorGroups = buildEditorSeamGroups();  
  
  for (List<Integer> group : editorGroups.values()) {  
    boolean createdOne = false;  
  
    for (int idx : group) {  
      SpatialObject pointObj = myObject.findChildObject("ponto-" + idx);  
      if (pointObj == null) continue;  
  
      ModelRenderer mr = pointObj.findComponent(ModelRenderer.class);  
  
      if (Enable_Visible_Points) {  
        if (!createdOne) {  
          if (mr == null) {  
            ModelRenderer newMr = new ModelRenderer();  
            Material mt = new Material();  
            Vertex vtx = new Vertex().loadPrimitive(0);  
  
            newMr.setVertex(vtx);  
            newMr.setMaterial(mt);  
  
            pointObj.addComponent(newMr);  
          }  
          createdOne = true;  
        } else {  
          if (mr != null) {  
            pointObj.removeComponent(mr);  
          }  
        }  
      } else {  
        if (mr != null) {  
          pointObj.removeComponent(mr);  
        }  
      }  
    }  
  }  
}  
  
  
private HashMap<String, List<Integer>> buildWorldSeamGroups() {  
    HashMap<String, List<Integer>> groups = new HashMap<String, List<Integer>>();  
    float threshold = 0.0001f; // tolerância pra agrupar posições próximas  
  
    for (int i = 0; i < points.length; i++) {  
        Vector3 worldPos = points[i].position; // posição mundial do ponto  
        // Arredonda cada componente para a tolerância definida  
        float x = Math.round(worldPos.getX() / threshold) * threshold;  
        float y = Math.round(worldPos.getY() / threshold) * threshold;  
        float z = Math.round(worldPos.getZ() / threshold) * threshold;  
  
        String key = String.format("%.6f_%.6f_%.6f", x, y, z);  
  
        if (!groups.containsKey(key)) {  
            groups.put(key, new ArrayList<Integer>());  
        }  
        groups.get(key).add(i);  
    }  
  
    return groups;  
}  
private void propagateFixedPointsInGroups() {  
    for (List<Integer> group : seamGroups.values()) {  
        boolean anyFixed = false;  
        for (int idx : group) {  
            if (points[idx].isFixed) {  
                anyFixed = true;  
                break;  
            }  
        }  
        if (anyFixed) {  
            for (int idx : group) {  
                points[idx].isFixed = true;  
            }  
        }  
    }  
}  
  
private void doResetPositionsFromOriginalMesh() {  
  ModelRenderer mr = myObject.findComponent(ModelRenderer.class);  
  if (mr != null) {  
    Vertex mesh = mr.getVertex();  
    NativeFloatBuffer originalVertexBuf = mesh.getVerticesBuffer();  
    int totalFloats = originalVertexBuf.capacity();  
    int vertexCount = totalFloats / 3;  
  
    for (int i = 0; i < vertexCount; i++) {  
      int idx = i * 3;  
      Vector3 originalLocalPos = new Vector3(originalVertexBuf.get(idx), originalVertexBuf.get(idx + 1), originalVertexBuf.get(idx + 2));  
      Vector3 worldPos = myObject.getTransform().transformPoint(originalLocalPos);  
  
      SpatialObject pointObj = myObject.findChildObject("ponto-" + i);  
      if (pointObj != null) {  
        pointObj.setPosition(myObject.getTransform().inverseTransformPoint(worldPos));  
      }  
  
      if (points != null && i < points.length && points[i] != null) {  
        points[i].position = worldPos.copy();  
        points[i].previousPosition = worldPos.copy();  
        points[i].localPosition = originalLocalPos;  
      }  
    }  
  
    Update_Mesh = false; // Força atualizar mesh com as novas posições  
  }  
  Reset_Points_Positions = false;  
}  
  
  
  
// mexer no editor  
public void stoppedRepeat() {  
  if (Enable_Visible_Points != lastEVisiblePoints) {  
    updateEditorPointRenderersOptimized();  
    lastEVisiblePoints = Enable_Visible_Points;  
  }  
  
  if (Reset_Points_Positions) {  
    doResetPositionsFromOriginalMesh();   
  }  
  
  if (myObject.getChildCount() > 0) {  
    SpatialObject obj = myObject.findChildObject("ponto-" + 0);  
    if (obj != null) Editor_Mode = true;  
    atualizado = 1;  
  }  
  
  if (Editor_Mode) {  
    // === INICIALIZAÇÃO ===  
    if (atualizado == 0) {  
      ModelRenderer mr = myObject.findComponent(ModelRenderer.class);  
      mesh = mr.getVertex();  
      NativeFloatBuffer originalVertexBuf = mesh.getVerticesBuffer();  
      int totalFloats = originalVertexBuf.capacity();  
      int vertexCount = totalFloats / 3;  
  
      vertexBuffer = new NativeFloatBuffer(totalFloats, NativeFloatBuffer.Precision.FLOAT16);  
      for (int i = 0; i < totalFloats; i++) {  
        vertexBuffer.put(originalVertexBuf.get(i));  
      }  
      mesh.setVertices(vertexBuffer); // opcional  
  
      points = new clothPoint[vertexCount];  
  
      for (int i = 0; i < vertexCount; i++) {  
        int b = i * 3;  
        Vector3 local = new Vector3(vertexBuffer.get(b), vertexBuffer.get(b + 1), vertexBuffer.get(b + 2));  
        Vector3 world = myObject.getTransform().transformPoint(local);  
  
        SpatialObject pobj = new SpatialObject("ponto-" + i);  
  
        clothPoint p = new clothPoint();  
        pobj.addComponent(p);  
        pobj.setGlobalScale(new Vector3(0.05f, 0.05f, 0.05f));  
      
        p.position = world;  
        p.previousPosition = world.copy();  
        p.localPosition = local;  
        p.vertexIndex = i;  
        pobj.setPosition(myObject.getTransform().inverseTransformPoint(p.position));  
        pobj.setParent(myObject);  
  
        points[i] = p;  
      }  
  
      atualizado = 1;  
    }  
  
    // === ATUALIZAÇÃO VISUAL ===  
    if (!Update_Mesh && points != null) {  
      ModelRenderer mr = myObject.findComponent(ModelRenderer.class);  
      mesh = mr.getVertex();  
      NativeFloatBuffer originalVertexBuf = mesh.getVerticesBuffer();  
      int totalFloats = originalVertexBuf.capacity();  
      int vertexCount = totalFloats / 3;  
  
      if (vertexBuffer == null) {  
        vertexBuffer = new NativeFloatBuffer(totalFloats, NativeFloatBuffer.Precision.FLOAT16);  
        for (int i = 0; i < totalFloats; i++) {  
          vertexBuffer.put(originalVertexBuf.get(i));  
        }  
        mesh.setVertices(vertexBuffer);  
      }  
  
      if (tempVerts == null || tempVerts.length != totalFloats) {  
        tempVerts = new float[totalFloats];  
      }  
  
      for (int i = 0; i < vertexCount; i++) {  
        SpatialObject pobj = myObject.findChildObject("ponto-" + i);  
        if (pobj != null) {  
          clothPoint p = pobj.findComponent(clothPoint.class);  
          p.position = pobj.getGlobalPosition();  
          p.previousPosition = pobj.getGlobalPosition();  
        }  
      }  
  
      for (int i = 0; i < vertexCount; i++) {  
        Vector3 local = myObject.getTransform().inverseTransformPoint(points[i].position);  
        int idx = i * 3;  
        tempVerts[idx] = local.getX();  
        tempVerts[idx + 1] = local.getY();  
        tempVerts[idx + 2] = local.getZ();  
      }  
  
      vertexBuffer.set(tempVerts);  
      meshDirty = true;   
      Update_Mesh = true;  
    }  
  }  
  
  if (Editor_Clear_All) {  
    // === SAÍDA DO MODO EDITOR ===  
    List<SpatialObject> toDestroy = new ArrayList<SpatialObject>();   
    for (int i = 0; i < myObject.getChildCount(); i++) {  
      SpatialObject pobj = myObject.findChildObject("ponto-" + i);  
      if (pobj != null) toDestroy.add(pobj);  
    }  
    for (SpatialObject p : toDestroy) p.destroy();  
    Editor_Mode = false;  
    Editor_Clear_All = false;  
    atualizado = 0;  
  }  
  
  if (meshDirty) {  
    mesh.recalculateBoundingBox();  
    mesh.regenerateNormals();  
    mesh.apply();  
    meshDirty = false;  
  }  
}  
  
@Override  
public void start() {  
  
  if (Stiffness > 1) Stiffness = 1;  
  // carrega mesh e buffers  
  ModelRenderer mr = myObject.findComponent(ModelRenderer.class);  
  mesh = mr.getVertex();  
  NativeFloatBuffer originalVertexBuf = mesh.getVerticesBuffer();  
  NativeFloatBuffer originalNormalBuf = mesh.getNormalsBuffer();  
  
  int totalFloats = originalVertexBuf.capacity(); // ex: 3000 floats = 1000 vértices  
  int totalNormals = originalNormalBuf.capacity();  
  int vertexCount = totalFloats / 3;  
  
  tempVerts = new float[vertexCount * 3];  
  vertexBuffer = new NativeFloatBuffer(totalFloats, NativeFloatBuffer.Precision.FLOAT16); // otimizado  
  normalBuffer = new NativeFloatBuffer(totalNormals, NativeFloatBuffer.Precision.FLOAT16); // otimizado  
  
  for (int i = 0; i < totalFloats; i++) {  
    vertexBuffer.put(originalVertexBuf.get(i));  
    normalBuffer.put(originalNormalBuf.get(i));  
  }  
  
  mesh.setVertices(vertexBuffer);  
  mesh.setNormals(normalBuffer);  
  
  // cria pontos  
  points = new clothPoint[vertexCount];  
  spatialHash = new SpatialHash(cellSize);  
  
  float minX = Float.POSITIVE_INFINITY, maxX = Float.NEGATIVE_INFINITY;  
  float minZ = Float.POSITIVE_INFINITY, maxZ = Float.NEGATIVE_INFINITY;  
  float threshold = 0.001f;  
  
  for (int i = 0; i < vertexCount; i++) {  
    int b = i * 3;  
    float x = vertexBuffer.get(b);  
    float z = vertexBuffer.get(b + 2);  
  
    if (x < minX) minX = x;  
    if (x > maxX) maxX = x;  
    if (z < minZ) minZ = z;  
    if (z > maxZ) maxZ = z;  
  }  
  for (int i = 0; i < vertexCount; i++) {  
    int b = i * 3;  
    Vector3 local = new Vector3(vertexBuffer.get(b), vertexBuffer.get(b + 1), vertexBuffer.get(b + 2));  
    Vector3 world = myObject.getTransform().transformPoint(local);  
  
    SpatialObject pobj = myObject.findChildObject("ponto-" + i);  
    clothPoint p;  
  
    if (pobj != null && pobj.findComponent(clothPoint.class) != null) {  
      // Usa o ponto já existente (pré-moldado no editor)  
      p = pobj.findComponent(clothPoint.class);  
      p.position = pobj.getGlobalPosition();  
      p.previousPosition = pobj.getGlobalPosition();  
      p.localPosition = myObject.getTransform().inverseTransformPoint(p.position);  
      p.vertexIndex = i;  
    } else {  
      // Cria um ponto novo  
      p = new clothPoint();  
      p.position = world;  
      p.previousPosition = world.copy();  
      p.localPosition = local;  
      p.vertexIndex = i;  
  
      // Pode aplicar fixação automática, se quiser  
      boolean fix = false;  
      if (Fix_Left && Math.abs(local.getX() - minX) < threshold) fix = true;  
      if (Fix_Right && Math.abs(local.getX() - maxX) < threshold) fix = true;  
      if (Fix_Top && Math.abs(local.getZ() - maxZ) < threshold) fix = true;  
      if (Fix_Bottom && Math.abs(local.getZ() - minZ) < threshold) fix = true;  
      // Depois que todos os pontos têm isFixed definido, propaga a fixação nos grupos seamGroups  
      p.isFixed = fix;  
    }  
  
    points[i] = p;  
    p.applyForce(new Vector3(0.01f, 0.01f, 0.01f));  
  
  }  
  
  // agrupa vértices coincidentes (seams)  
     seamGroups=buildWorldSeamGroups();  
      propagateFixedPointsInGroups();  
  
  // cria constraints de costura (opcional)  
  if (Seam_Constraints) {  
    for (List<Integer> group : seamGroups.values()) {  
      if (group.size() < 2) continue;  
      int first = group.get(0);  
      for (int k = 1; k < group.size(); k++) {  
        addConstraint(points[first], points[group.get(k)], 0.1f, ClothConstraint.Type.STRUCTURAL);  
      }  
    }  
  }  
           
         
  // cria structural constraints pelos triângulos  
  NativeIntBuffer triBuf = mesh.getTrianglesBuffer();  
  Set<String> seenEdges = new HashSet<String>();  
  for (int ti = 0; ti < triBuf.capacity(); ti += 3) {  
    int a = triBuf.get(ti);  
    int b = triBuf.get(ti + 1);  
    int c = triBuf.get(ti + 2);  
    addEdgeConstraint(a, b, seenEdges);  
    addEdgeConstraint(b, c, seenEdges);  
    addEdgeConstraint(c, a, seenEdges);  
  }  
  
  for (SpatialObject obj : Colliders_Mesh) {  
    MeshCollider mc = new MeshCollider();  
    mc.init(obj);  
    meshColliders.add(mc);  
  }  
  // cria bending constraints  
  createBendingConstraints();  
  
  for (int i = 0; i < myObject.getChildCount(); i++) {  
    ModelRenderer model = myObject.getChildAt(i).findComponent(ModelRenderer.class);  
  
    if (model == null) continue;  
  
    myObject.getChildAt(i).removeComponent(model);  
  }  
  
  // normais e bounding box iniciais  
  mesh.regenerateNormals();  
  mesh.recalculateBoundingBox();  
  mesh.apply();  
  
  // CHANGED: sensible default for dragCoefficient if user didn't set it  
  if (Drag_Coefficient == 0f) Drag_Coefficient = 1.0f; // CHANGED  
}  
  
/////////////  
  
void updateFixedPoints() {  
  float minX = Float.POSITIVE_INFINITY, maxX = Float.NEGATIVE_INFINITY;  
  float minZ = Float.POSITIVE_INFINITY, maxZ = Float.NEGATIVE_INFINITY;  
  
  for (clothPoint p : points) {  
    float x = p.localPosition.getX();  
    float z = p.localPosition.getZ();  
    if (x < minX) minX = x;  
    if (x > maxX) maxX = x;  
    if (z < minZ) minZ = z;  
    if (z > maxZ) maxZ = z;  
  }  
  
  float threshold = 0.001f;  
  
  for (clothPoint p : points) {  
    boolean fix = false;  
    if (Fix_Left && Math.abs(p.localPosition.getX() - minX) < threshold) fix = true;  
    if (Fix_Right && Math.abs(p.localPosition.getX() - maxX) < threshold) fix = true;  
    if (Fix_Top && Math.abs(p.localPosition.getZ() - maxZ) < threshold) fix = true;  
    if (Fix_Bottom && Math.abs(p.localPosition.getZ() - minZ) < threshold) fix = true;  
    p.isFixed = fix;  
  }  
}  
  
///////////  
  
@Override
public void repeat() {
  if (Update_Fixed_Pos) {
    updateFixedPoints();
    Update_Fixed_Pos = false;
  }

  frameCount++;
  float dt = Time.deltaTime();
  windTimeAccumulator += dt;

  // accumulate time and run fixed physics steps
  physAccumulator += dt;
  float fixedStep = timeStep; // 0.016f por padrão
  int maxStepsPerFrame = 5; // limita para evitar stall

  int steps = 0;
  while (physAccumulator >= fixedStep && steps < maxStepsPerFrame) {
    // physics sub-step start
    spatialHash.clear();
    for (clothPoint p : points) spatialHash.insert(p, p.position);

    if (!Do_Not_Use_Phys) {
      Vector3 gravityForce = new Vector3(0, Gravity, 0);

      // apply forces
      for (clothPoint p : points) {
        if (p.isFixed || p.anchored) continue;
        p.applyForce(gravityForce);
        if (Advanced_Wind) applyWind(p);
      }

      // integração Verlet — use velocityDamping explícito
      float damping = Velocity_Damping;
      for (clothPoint p : points) p.simulate(fixedStep, damping);
    }

    // constraint iterations
    if (Quality_Level_Max2 > 2) Quality_Level_Max2 = 2;

    int iter = (Quality_Level_Max2 == 2 ? 20 : Quality_Level_Max2 == 1 ? 5 : 3);
    for (int k = 0; k < iter; k++) {
      for (ClothConstraint c : constraints) c.satisfy();
      if (Self_Collision && Quality_Level_Max2 > 0) solveSelfCollision();
    }
   
 

    // colisões
    if (Colliders_Box.size() > 0) collideWithBoxes();
    if (Colliders_Sphere.size() > 0) collideWithSpheres();
    if (Mesh_Collision && Colliders_Mesh.size() > 0) collideWithMeshVertices();

    // welded seams (note: pode amortecer movimento)
    // agora ignoramos grupos que contém vértices rasgados (para manter tear visível)
    for (List<Integer> group : seamGroups.values()) {
      if (group.size() < 2) continue;

      Vector3 avgPos = new Vector3(0, 0, 0);
      Vector3 avgPrev = new Vector3(0, 0, 0);
      for (int idx : group) {
        avgPos = avgPos.sum(points[idx].position);
        avgPrev = avgPrev.sum(points[idx].previousPosition);
      }
      avgPos = avgPos.mul(1.0f / group.size());
      avgPrev = avgPrev.mul(1.0f / group.size());
      for (int idx : group) {
        points[idx].position = avgPos.copy();
        points[idx].previousPosition = avgPrev.copy();
      }
    }

    updateVertexBuffer();

    physAccumulator -= fixedStep;
    steps++;
    // physics sub-step end
  }

  // safety: prevent huge backlog
  if (physAccumulator > 0.25f) physAccumulator = 0.25f;

  // --- apply mesh if dirty (runtime-safe, once per frame) ---
  if (meshDirty) {
    framesSinceNormals++;

    boolean shouldRecalcNormals = forceRecalculateNormals || (framesSinceNormals >= Math.max(1, normalsUpdateInterval));

    // Antes de recalcular normais, cheque se o normalBuffer/vertexBuffer tem dados válidos (opcional)
    // sanitize normalBuffer if needed (similar approach).

    if (shouldRecalcNormals) {
      // evita chamadas desnecessárias que podem causar flicker em PBR/shadows
      mesh.regenerateNormals();
      framesSinceNormals = 0;
    }

    // recalcula bounding box sempre antes do apply (importante para shadow/occlusion)
    mesh.recalculateBoundingBox();

    // apply atualiza GPU / render thread; faça isso só aqui, em um ponto estável
    mesh.apply();

    meshDirty = false;
  }

  // tempo real de vento (fora dos substeps já ok)
}
////////////    
void createBendingConstraints() {  
  NativeIntBuffer triBuf = mesh.getTrianglesBuffer();  
  Map<String, int[]> edgeToTriangles = new HashMap<String, int[]>();  
  
  // Mapeia cada aresta aos triângulos que a contêm  
  for (int ti = 0; ti < triBuf.capacity(); ti += 3) {  
    int[] tri = {triBuf.get(ti), triBuf.get(ti + 1), triBuf.get(ti + 2)};  
    for (int i = 0; i < 3; i++) {  
      int a = tri[i];  
      int b = tri[(i + 1) % 3];  
      int min = Math.min(a, b);  
      int max = Math.max(a, b);  
      String key = min + "_" + max;  
  
      if (!edgeToTriangles.containsKey(key)) {  
        edgeToTriangles.put(key, new int[] {ti});  
      } else {  
        int[] prev = edgeToTriangles.get(key);  
        edgeToTriangles.put(key, new int[] {prev[0], ti});  
      }  
    }  
  }  
  
  
  
  
    



  // Para cada aresta compartilhada por dois triângulos, cria uma bending constraint  
  for (Map.Entry<String, int[]> entry : edgeToTriangles.entrySet()) {  
    int[] tris = entry.getValue();  
    if (tris.length == 2) {  
      int t1 = tris[0];  
      int t2 = tris[1];  
  
      int[] tri1 = {triBuf.get(t1), triBuf.get(t1 + 1), triBuf.get(t1 + 2)};  
      int[] tri2 = {triBuf.get(t2), triBuf.get(t2 + 1), triBuf.get(t2 + 2)};  
  
      // Encontra vértices opostos (que não estão na aresta compartilhada)  
      int[] shared = getSharedEdge(tri1, tri2);  
      if (shared != null) {  
        int opp1 = getOppositeVertex(tri1, shared[0], shared[1]);  
        int opp2 = getOppositeVertex(tri2, shared[0], shared[1]);  
  
        if (opp1 != -1 && opp2 != -1) {  
          addConstraint(  
              points[opp1],  
              points[opp2],  
              Bending_Constraints_Stiff, // stiffness da dobra (ajustável)  
              ClothConstraint.Type.BENDING);  
        }  
      }  
    }  
  }  
}  
  
///////////  
  
// Encontra a aresta compartilhada entre dois triângulos  
private int[] getSharedEdge(int[] tri1, int[] tri2) {  
  int sharedCount = 0;  
  int[] shared = new int[2];  
  
  for (int v1 : tri1) {  
    for (int v2 : tri2) {  
      if (v1 == v2) {  
        if (sharedCount < 2) {  
          shared[sharedCount++] = v1;  
        }  
      }  
    }  
  }  
  return (sharedCount == 2) ? shared : null;  
}  
  
/////////  
  
// Retorna o vértice do triângulo que não está na aresta compartilhada  
private int getOppositeVertex(int[] tri, int e1, int e2) {  
  for (int v : tri) {  
    if (v != e1 && v != e2) {  
      return v;  
    }  
  }  
  return -1;  
}  
  
//////////  
  
private void addEdgeConstraint(int i1, int i2, Set<String> seen) {  
  int min = Math.min(i1, i2), max = Math.max(i1, i2);  
  String key = min + "_" + max;  
  if (!seen.contains(key)) {  
    seen.add(key);  
    addConstraint(points[min], points[max], Edge_Constraints_Stiff, ClothConstraint.Type.STRUCTURAL);  
  }  
}  
  
//////////  
  
private void addConstraint(clothPoint p1, clothPoint p2, float stiffness, ClothConstraint.Type type) {  
  ClothConstraint c = new ClothConstraint();  
  c.p1 = p1;  
  c.p2 = p2;  
  c.stiffness = stiffness;  
  c.type = type;  
  c.start();  
constraints.add(c);
constraintRestLengths.put(c, c.p1.position.distance(c.p2.position));
}  
  
////////////  
  
private void applyWind(clothPoint p) {  
  int ni = p.vertexIndex * 3;  
  tempVec1.set(normalBuffer.get(ni), normalBuffer.get(ni + 1), normalBuffer.get(ni + 2));  
  tempVec1.normalize();  
  
  // ruído e direção variável  
  float nx = simpleNoise(p.position.getX() * Wind_Noise_Scale, 0, windTimeAccumulator * Wind_Frequency);  
  float nz = simpleNoise(0, p.position.getZ() * Wind_Noise_Scale, windTimeAccumulator * Wind_Frequency);  
  
  // CHANGED: reuse windTemp and relVelTemp to avoid allocations  
  windTemp.set(Wind_Global_Direction.getX() + nx * Turbulence * 0.3f,  
               Wind_Global_Direction.getY() + (nx + nz) * 0.15f * Turbulence,  
               Wind_Global_Direction.getZ() + nz * Turbulence * 0.3f);  
  windTemp.normalize();  
  
  float facing = Math.max(0, tempVec1.dot(windTemp));  
  float pressure = facing * facing * Pressure_Coefficient * Wind_Strength;  
  
  relVelTemp.set(p.position.getX() - p.previousPosition.getX(),  
                 p.position.getY() - p.previousPosition.getY(),  
                 p.position.getZ() - p.previousPosition.getZ());  
  relVelTemp.div(Time.deltaTime());  
  
  Vector3 windVel = windTemp.mul(Wind_Strength);  
  float drag = Math.max(0, windVel.sub(relVelTemp).dot(windTemp)) * Drag_Coefficient;  
  
  float var = 1.0f + nx * 0.2f;  
  Vector3 f = windTemp.mul((pressure + drag) * var);  
  p.applyForce(f);  
}  
  
//////////////  
  
private float simpleNoise(float x, float y, float z) {  
  float v = (float) Math.sin(x * 12.9898f + y * 78.233f + z * 45.164f) * 43758.5453f;  
  return v - (int) v;  
}  
  
////////////  
  
private void solveSelfCollision() {  
  float minD = cellSize * 0.5f;  
  float minDSq = minD * minD;  
  for (clothPoint p1 : points) {  
    List<clothPoint> near = spatialHash.query(p1.position, minD);  
    for (clothPoint p2 : near) {  
      if (p1 == p2) continue;  
      Vector3 d = p2.position.sub(p1.position);  
      float dsq = d.dot(d);  
      if (dsq > 0 && dsq < minDSq) {  
        float dist = (float) Math.sqrt(dsq);  
        float diff = (minD - dist) / dist;  
        Vector3 corr = d.mul(0.5f * diff);  
        if (!p1.isFixed && !p1.anchored) p1.position = p1.position.sub(corr);  
        if (!p2.isFixed && !p1.anchored) p2.position = p2.position.sum(corr);  
      }  
    }  
  }  
}  
  
/////////////  
  
private void collideWithBoxes() {  
  Vector3 center = calculateClothCenter();  
  for (SpatialObject obj : Colliders_Box) {  
    Vector3 pos = obj.getTransform().getGlobalPosition();  
    //  if (center.distance(pos) > 5f) continue;  
    Vector3 half = obj.getTransform().getGlobalScale().mul(0.52f);  
    Vector3 min = pos.sub(half), max = pos.sum(half);  
    for (clothPoint p : points) {  
      if (p.isFixed) continue;  
      Vector3 pp = p.position;  
      boolean inX = pp.getX() >= min.getX() && pp.getX() <= max.getX();  
      boolean inY = pp.getY() >= min.getY() && pp.getY() <= max.getY();  
      boolean inZ = pp.getZ() >= min.getZ() && pp.getZ() <= max.getZ();  
      if (inX && inY && inZ) {  
        float dxMin = Math.abs(pp.getX() - min.getX()), dxMax = Math.abs(max.getX() - pp.getX());  
        float dyMin = Math.abs(pp.getY() - min.getY()), dyMax = Math.abs(max.getY() - pp.getY());  
        float dzMin = Math.abs(pp.getZ() - min.getZ()), dzMax = Math.abs(max.getZ() - pp.getZ());  
        // empurra pela face mais próxima  
        float md = dxMin;  
        Vector3 corr = new Vector3(-dxMin, 0, 0);  
        if (dxMax < md) {  
          md = dxMax;  
          corr.set(dxMax, 0, 0);  
        }  
        if (dyMin < md) {  
          md = dyMin;  
          corr.set(0, -dyMin, 0);  
        }  
        if (dyMax < md) {  
          md = dyMax;  
          corr.set(0, dyMax, 0);  
        }  
        if (dzMin < md) {  
          md = dzMin;  
          corr.set(0, 0, -dzMin);  
        }  
        if (dzMax < md) {  
          md = dzMax;  
          corr.set(0, 0, dzMax);  
        }  
        p.position = p.position.sum(corr);  
        // damping  
        Vector3 vel = p.position.sub(p.previousPosition).mul(0.5f);  
        p.previousPosition = p.position.sub(vel);  
      }  
    }  
  }  
}  
  
/////////////  
  private void collideWithSpheres() {
  for (SpatialObject obj : Colliders_Sphere) {
    Vector3 pos = obj.getTransform().getGlobalPosition();
    float r = obj.getTransform().getGlobalScale().getX() * 0.53f;
    float effectiveR = r + pointRadius;
    float thrSq = effectiveR * effectiveR;

    for (clothPoint p : points) {
      if (p.isFixed) continue;
      Vector3 d = p.position.sub(pos);
      float distSq = d.dot(d);
      if (distSq < thrSq && distSq > 1e-8f) {
        float dist = (float) Math.sqrt(distSq);
        Vector3 n = d.mul(1.0f / dist); // safe because dist > 1e-8
        float penetration = effectiveR - dist;
        Vector3 corr = n.mul(penetration);
        p.position = p.position.sum(corr);
        // damp previousPosition proportional to correction (keeps velocity sane)
        Vector3 vel = p.position.sub(p.previousPosition).mul(0.5f);
        p.previousPosition = p.position.sub(vel);
      } else if (distSq <= 1e-8f) {
        // caso muito perto/mesmo local: empurra numa direção fixa + random pequena
        Vector3 corr = new Vector3(0, effectiveR, 0);
        p.position = p.position.sum(corr);
        p.previousPosition = p.position.copy();
      }
    }
  }
} 
  
//////////////  
  
private void collideWithMeshVertices() {  
  float threshold = 0.6f; // ajuste fino  
  float thrSq = threshold * threshold;  
  
  for (MeshCollider mc : meshColliders) {  
    for (clothPoint p : points) {  
      if (p.isFixed) continue;  
      for (Vector3 localV : mc.localVerts) {  
        // posição do vértice do objeto em world space  
        Vector3 wv = mc.transform.transformPoint(localV);  
        Vector3 delta = p.position.sub(wv);  
        float distSq = delta.dot(delta);  
        if (distSq < thrSq && distSq > 0f) {  
          float dist = (float) Math.sqrt(distSq);  
          float diff = (threshold - dist) / dist;  
          Vector3 corr = delta.mul(diff * 0.5f);  
          // empurra para fora  
          p.position = p.position.sum(corr);  
          // damping simples  
          Vector3 vel = p.position.sub(p.previousPosition).mul(0.5f);  
          p.previousPosition = p.position.sub(vel);  
        }  
      }  
    }  
  }  
}  
  
/////////////  
  
private void updateVertexBuffer() {
  for (int i = 0; i < points.length; i++) {
    Vector3 local = myObject.getTransform().inverseTransformPoint(points[i].position);
    int idx = i * 3;
    tempVerts[idx] = local.getX();
    tempVerts[idx + 1] = local.getY();
    tempVerts[idx + 2] = local.getZ();
  }

  // safety: remove NaN/Inf antes de enviar ao GPU
  sanitizeTempVerts(tempVerts, vertexBuffer);

  vertexBuffer.set(tempVerts);
  meshDirty = true;
}  
private Vector3 calculateClothCenter() {  
  Vector3 c = new Vector3();  
  for (clothPoint p : points) c = c.sum(p.position);  
  return c.div(points.length);  
}  
  
////////////////////  
//pure GPT , idk if this really help that much,but,better than nothing i think  
// SpatialHash otimizado com HashMap  
class SpatialHash {  
  private Map<Integer, HashCell> cells = new HashMap<Integer, HashCell>();  
  private float cellSize;  
  
  SpatialHash(float cs) {  
    cellSize = cs;  
  }  
  
  void clear() {  
    cells.clear();  
  }  
  
  void insert(clothPoint p, Vector3 pos) {  
    int x = (int) (pos.getX() / cellSize);  
    int y = (int) (pos.getY() / cellSize);  
    int z = (int) (pos.getZ() / cellSize);  
    int h = (x * 92837111) ^ (y * 689287499) ^ (z * 283923481);  
  
    HashCell cell = cells.get(h);  
    if (cell == null) {  
      cell = new HashCell();  
      cells.put(h, cell);  
    }  
    cell.points.add(p);  
  }  
  
  List<clothPoint> query(Vector3 pos, float r) {  
    List<clothPoint> out = new ArrayList<clothPoint>();  
    int minX = (int) ((pos.getX() - r) / cellSize), maxX = (int) ((pos.getX() + r) / cellSize);  
    int minY = (int) ((pos.getY() - r) / cellSize), maxY = (int) ((pos.getY() + r) / cellSize);  
    int minZ = (int) ((pos.getZ() - r) / cellSize), maxZ = (int) ((pos.getZ() + r) / cellSize);  
    for (int x = minX; x <= maxX; x++)  
      for (int y = minY; y <= maxY; y++)  
        for (int z = minZ; z <= maxZ; z++) {  
          int h = (x * 92837111) ^ (y * 689287499) ^ (z * 283923481);  
          HashCell cell = cells.get(h);  
          if (cell != null) out.addAll(cell.points);  
        }  
    return out;  
  }  
}  
  
class HashCell {  
  List<clothPoint> points = new ArrayList<clothPoint>();  
}  
