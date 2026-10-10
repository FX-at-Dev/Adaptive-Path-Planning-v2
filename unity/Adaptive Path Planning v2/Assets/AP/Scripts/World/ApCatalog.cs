








using System;
using UnityEngine;

namespace Ap
{
    [CreateAssetMenu(menuName = "Adaptive Path Planning/Catalog", fileName = "Catalog")]
    public class ApCatalog : ScriptableObject
    {
        [Serializable]
        public class Variant
        {
            public GameObject prefab;
            [Min(0f)] public float weight = 1f;
        }

        [Serializable]
        public class Entry
        {
            [Tooltip("Road users: the class (car, bus, truck, auto, two_wheeler, bicycle, pedestrian, cattle, pushcart, static). Scenery: the kind (house, tree, wall, pole, field, cone, barrier, sign, milestone).")]
            public string key;
            public Variant[] variants = new Variant[0];
        }

        [Tooltip("One entry per road-user class.")]
        public Entry[] roadUsers = new Entry[0];
        [Tooltip("One entry per kind of scenery and road object.")]
        public Entry[] scenery = new Entry[0];
        [Tooltip("Used for a road-user class with no entry.")]
        public GameObject fallbackRoadUser;

        public GameObject RoadUser(string cls, System.Random rng) => Pick(roadUsers, cls, rng) ?? fallbackRoadUser;
        public GameObject Scenery(string kind, System.Random rng) => Pick(scenery, kind, rng);


        public static GameObject Pick(Entry[] list, string key, System.Random rng)
        {
            if (list == null) return null;
            foreach (var e in list)
            {
                if (e == null || e.key != key || e.variants == null) continue;
                float total = 0f;
                foreach (var v in e.variants) if (v != null && v.prefab != null) total += Mathf.Max(0f, v.weight);
                if (total <= 0f) return null;
                double r = rng.NextDouble() * total;
                foreach (var v in e.variants)
                {
                    if (v == null || v.prefab == null) continue;
                    r -= Mathf.Max(0f, v.weight);
                    if (r <= 0) return v.prefab;
                }
                foreach (var v in e.variants) if (v != null && v.prefab != null) return v.prefab;
            }
            return null;
        }
    }
}
