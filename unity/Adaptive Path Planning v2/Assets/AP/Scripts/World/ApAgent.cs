



using UnityEngine;

namespace Ap
{
    public class ApAgent : MonoBehaviour
    {
        [Tooltip("Agent id in the run file (frames[].agent_id).")]
        public int agentId;

        [Tooltip("Road-user class: car, bus, truck, auto, two_wheeler, bicycle, pedestrian, cattle, pushcart or static.")]
        public string agentClass = "car";
    }
}
