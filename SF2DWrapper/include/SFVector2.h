namespace SF2D 
{
	public value class SFVector2
	{
	public:
		float X;
		float Y;

		SFVector2(float x, float y);

		SFVector2 operator +(SFVector2 other);
		SFVector2 operator * (float num);
		SFVector2 operator +(float num);		
	};
}