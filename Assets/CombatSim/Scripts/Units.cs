namespace CombatSim
{
    public static class Units
    {
        public const float MillimetresPerMetre = 1000f;

        public static float MillimetresToMetres(float millimetres)
        {
            return millimetres / MillimetresPerMetre;
        }

        public static float MetresToMillimetres(float metres)
        {
            return metres * MillimetresPerMetre;
        }
    }
}
