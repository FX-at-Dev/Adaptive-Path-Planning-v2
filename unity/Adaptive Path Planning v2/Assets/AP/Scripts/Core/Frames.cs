







using UnityEngine;

namespace Ap
{
    public static class Frames
    {
        public static Vector3 Pos(float x, float y, float h = 0f) => new Vector3(x, h, y);

        public static float YawDeg(float psi) => 90f - psi * Mathf.Rad2Deg;

        public static Quaternion Rot(float psi) => Quaternion.Euler(0f, YawDeg(psi), 0f);


        public static Vector3 EgoCentre(float x, float y, float psi, float rearToCentre, float h = 0f)
        {
            return Pos(x + Mathf.Cos(psi) * rearToCentre, y + Mathf.Sin(psi) * rearToCentre, h);
        }


        public static float LerpAngle(float a, float b, float u)
        {
            float d = Mathf.Repeat(b - a + Mathf.PI, 2f * Mathf.PI) - Mathf.PI;
            return a + d * u;
        }
    }
}
