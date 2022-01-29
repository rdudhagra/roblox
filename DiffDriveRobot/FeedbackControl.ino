#define FBK_lambda 0.1
#define FBK_kX (1.0/FBK_lambda) * 1
#define FBK_kY (2.0/(FBK_lambda*FBK_lambda))
#define FBK_kTh (2.0/FBK_lambda) * 0.4

extern float currentX;
extern float currentY;
extern float currentTh;

float ffX; // ff = feed-forward
float ffY;
float ffTh;

float lastFFTickTime;

void resetFF() {
  ffX = 0;
  ffY = 0;
  ffTh = 0;
  lastFFTickTime = getTime();
}

void ffTick(float V, float w) {
  float currentTime = getTime();
  float dt = currentTime - lastFFTickTime;

  float ds = V * dt;
  float dth = w * dt;

  float thetaNext = ffTh + dth / 2;
  ffX += ds * cos(thetaNext);
  ffY += ds * sin(thetaNext);
  ffTh += dth;
  ffTh = normalizeAngle(ffTh);

  lastFFTickTime = currentTime;
}

void add_feedback(float *V, float *w) {
  float errX, errY, errTh;
  calcError(ffX, ffY, ffTh, currentX, currentY, currentTh, &errX, &errY, &errTh);
  *V += FBK_kX * errX;
  *w += FBK_kY * errY + FBK_kTh * errTh;
}

void calcError(float ffX, float ffY, float ffTh, float currentX, float currentY, float currentTh, float *errX, float *errY, float *errTh) {
  float errXWorld = ffX - currentX;
  float errYWorld = ffY - currentY;

  *errX = errXWorld * cos(currentTh) + errYWorld * sin(currentTh);
  *errY = errXWorld * -sin(currentTh) + errYWorld * cos(currentTh);
  *errTh = normalizeAngle(ffTh - currentTh);
}
