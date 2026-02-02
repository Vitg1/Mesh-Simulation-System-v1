/* @Author Sr Mil Games */

Vector3 position;
Vector3 localPosition;
Vector3 previousPosition;
public boolean isFixed = false;
public int vertexIndex;
public SpatialObject anchor;
Vector3 accumulatedForce = new Vector3();
@Hide
boolean anchored;

public void applyForce(Vector3 force) {
  accumulatedForce = accumulatedForce.sum(force);
}


// No clothPoint.simulate:
public void simulate(float timeStep, float damping) {

  if (anchor != null && anchor.exists()) {
    anchored = true;
    localPosition=anchor.position;
    position = anchor.getGlobalPosition();
    previousPosition = anchor.getGlobalPosition();
    accumulatedForce.set(0,0,0);
    return;
  } else anchored = false;

  if (isFixed) {
    return;
  }
  Vector3 velocity = position.sub(previousPosition).mul(damping);

  Vector3 acceleration = accumulatedForce.mul(timeStep * timeStep);
  Vector3 next = position.sum(velocity).sum(acceleration);
  previousPosition = position;
  position = next;
  accumulatedForce.set(0, 0, 0);
}