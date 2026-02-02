
    public enum Type {
        STRUCTURAL,
        SHEAR,
        BENDING
    }

    public clothPoint p1;
    public clothPoint p2;
    public float restLength;
    public float stiffness;
    public Type type;
    // Inicializa o comprimento de descanso
    public void start() {
        restLength = p1.position.distance(p2.position);
    }

    // Aplica a constraint para satisfazer a restrição
    public void satisfy() {
        Vector3 delta = p2.position.sub(p1.position);
        float currentLength = delta.length();

        // Evita divisão por zero
        if (currentLength < 0.0001f) return;

        float diff = (currentLength - restLength) / currentLength;

        // Ajusta rigidez baseado no tipo
        float adjustment = stiffness;
        if (type == Type.SHEAR) {
            adjustment *= 0.6f;
        } else if (type == Type.BENDING) {
            adjustment *= 0.2f;
        }

        Vector3 correction = delta.mul(adjustment * diff);

        if (!p1.isFixed && !p1.anchored)p1.position = p1.position.sum(correction.mul(0.5f));
        if (!p2.isFixed && !p2.anchored) p2.position = p2.position.sub(correction.mul(0.5f));
    }

