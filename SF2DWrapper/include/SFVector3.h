namespace SF3D 
{
	public value class SFVector3
	{
	public:
		float X;
		float Y;
		float Z;

		SFVector3(float x, float y, float z);

		SFVector3 operator +(const SFVector3 &other);
		SFVector3 operator * (float num);
		SFVector3 operator +(float num);		
	};
}