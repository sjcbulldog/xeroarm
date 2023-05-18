#include "FabrikChain.h"

FabrikChain::FabrikChain()
{
}

bool FabrikChain::solveIK(const Translation2d& target)
{
	//
	// Loop from end bone back to the base of the arm
	//
	for (int loop = bones_.count() - 1; loop >= 0; loop--) 
	{
		const FabrikBone& bone = bones_.at(loop);
		double length = bone.length();

		if (loop != bones_.count() - 1)
		{
			const FabrikBone& outerBone = bones_.at(loop + 1);
			Translation2d outerBoneOutgerToInnerUV = outerBone.getDirectionUV().inverse();
			Translation2d thisBoneOuterToInnerUV = bone.getDirectionUV().inverse();

		}
		else
		{

		}
	}

	return false;
}
