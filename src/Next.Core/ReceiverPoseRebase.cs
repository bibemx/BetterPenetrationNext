namespace BetterPenetrationNext.Core
{
    public static class ReceiverPoseRebase
    {
        public static Vec3 Offset(Vec3 solved,Vec3 offset,Vec3 currentNative,Linear3 previousInverse,Vec3 previousOrigin,Linear3 current,Vec3 currentOrigin,double ownership)
        {
            Vec3 carriedOffset=current.Multiply(previousInverse.Multiply(offset));
            if(ownership<=0)return carriedOffset;
            Vec3 oldNative=solved-offset;
            Vec3 carriedNative=current.Multiply(previousInverse.Multiply(oldNative-previousOrigin))+currentOrigin;
            return carriedOffset+(carriedNative-currentNative)*ownership;
        }
    }
}
