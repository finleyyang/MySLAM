//
// Created by finley on 23-3-17.
//
#include "vision/ORBextractor.h"
#include <algorithm>
#include <opencv2/core/types.hpp>
#include <vector>

#define useopencv 0

namespace my_slam
{


	const int PATCH_SIZE = 31;     //<使用灰度质心法计算特征点的方向信息时，图像块的大小,或者说是直径
	const int HALF_PATCH_SIZE = 15;   //<上面这个大小的一半，或者说是半径
	const int EDGE_THRESHOLD = 19;   //<算法生成的图像边

	static float IC_Angle(const cv::Mat& image, cv::Point2f pt,  const std::vector<int> & u_max)
	{
		//图像的矩，前者是按照图像块的y坐标加权，后者是按照图像块的x坐标加权
		int m_01 = 0, m_10 = 0;

		//获得这个特征点所在的图像块的中心点坐标灰度值的指针center
		const uchar* center = &image.at<uchar> (cvRound(pt.y), cvRound(pt.x));

		// Treat the center line differently, v=0
		//这条v=0中心线的计算需要特殊对待
		//后面是以中心行为对称轴，成对遍历行数，所以PATCH_SIZE必须是奇数
		for (int u = -HALF_PATCH_SIZE; u <= HALF_PATCH_SIZE; ++u)
			//注意这里的center下标u可以是负的！中心水平线上的像素按x坐标（也就是u坐标）加权
			m_10 += u * center[u];

		// Go line by line in the circuI853lar patch
		//这里的step1表示这个图像一行包含的字节总数。参考[https://blog.csdn.net/qianqing13579/article/details/45318279]
		int step = (int)image.step1();
		//注意这里是以v=0中心线为对称轴，然后对称地每成对的两行之间进行遍历，这样处理加快了计算速度
		for (int v = 1; v <= HALF_PATCH_SIZE; ++v)
		{
			// Proceed over the two lines
			//本来m_01应该是一列一列地计算的，但是由于对称以及坐标x,y正负的原因，可以一次计算两行
			int v_sum = 0;
			// 获取某行像素横坐标的最大范围，注意这里的图像块是圆形的！
			int d = u_max[v];
			//在坐标范围内挨个像素遍历，实际是一次遍历2个
			// 假设每次处理的两个点坐标，中心线下方为(x,y),中心线上方为(x,-y)
			// 对于某次待处理的两个点：m_10 = Σ x*I(x,y) =  x*I(x,y) + x*I(x,-y) = x*(I(x,y) + I(x,-y))
			// 对于某次待处理的两个点：m_01 = Σ y*I(x,y) =  y*I(x,y) - y*I(x,-y) = y*(I(x,y) - I(x,-y))
			for (int u = -d; u <= d; ++u)
			{
				//得到需要进行加运算和减运算的像素灰度值
				//val_plus：在中心线下方x=u时的的像素灰度值
				//val_minus：在中心线上方x=u时的像素灰度值
				int val_plus = center[u + v*step], val_minus = center[u - v*step];
				//在v（y轴）上，2行所有像素灰度值之差
				v_sum += (val_plus - val_minus);
				//u轴（也就是x轴）方向上用u坐标加权和（u坐标也有正负符号），相当于同时计算两行
				m_10 += u * (val_plus + val_minus);
			}
			//将这一行上的和按照y坐标加权
			m_01 += v * v_sum;
		}

		// 为了加快速度使用了fastAtan2（）函数，输出为[0，360）精确度为0.3
		return cv::fastAtan2((float)m_01, (float)m_10);
	}

//乘数因子，一度对应着多少弧度
	const float factorPI = (float)(CV_PI/180.f);

/**
 * @brief 计算ORB特征点的描述子。注意这个是全局的静态函数，只能是在本文件内被调用
 * @param[in] kpt       特征点对象
 * @param[in] img       提取特征点的图像
 * @param[in] pattern   预定义好的采样模板
 * @param[out] desc     用作输出变量，保存计算好的描述子，维度为32*8 = 256 bit
 */
	static void computeOrbDescriptor(const cv::KeyPoint& kpt,
		const cv::Mat& img, const cv::Point* pattern,
		uchar* desc)
	{
		//在计算描述子的过程中，应该把特征走位像素旋转到主方向上来计算，这里为了编程方便实践上对pattern进行反向旋转,这里的角度就是特征点的重心方向
		float angle = (float)kpt.angle*factorPI;
		//计算这个角度的余弦值和正弦值
		float a = (float)cos(angle), b = (float)sin(angle);

		//获得图像中心指针
		const uchar* center = &img.at<uchar>(cvRound(kpt.pt.y), cvRound(kpt.pt.x));
		//获得图像的每行的字节数
		const int step = (int)img.step;

		//旋转公式
		//x'=xcos（thate）-ysin（thate）
		//y'=xsin（thate）+ycos（thate）
		//原始的BRIEF描述子没有方向不变性，通过加入关键点的方向来计算描述子，称之为Steer BRIEF，具有较好旋转不变特性
		//具体地，在计算的时候需要将这里选取的采样模板中点的x轴方向旋转到特征点的方向。
		//获得采样点中某个idx所对应的点的灰度值,这里旋转前坐标为(x,y), 旋转后坐标(x',y')，他们的变换关系:
		//下面表示 y'* step + x'
#define GET_VALUE(idx) \
        center[cvRound(pattern[idx].x*b + pattern[idx].y*a)*step + \
               cvRound(pattern[idx].x*a - pattern[idx].y*b)]

		//brief描述子由32*8位组成
		//其中每一位是来自于两个像素点灰度的直接比较，所以每比较出8bit结果，需要16个随机点，这也就是为什么pattern需要+=16的原因
		for (int i = 0; i < 32; ++i, pattern += 16)
		{
			int t0, t1, val;
			t0 = GET_VALUE(0); t1 = GET_VALUE(1);
			val = t0 < t1;
			t0 = GET_VALUE(2); t1 = GET_VALUE(3);
			val |= (t0 < t1) << 1;
			t0 = GET_VALUE(4); t1 = GET_VALUE(5);
			val |= (t0 < t1) << 2;
			t0 = GET_VALUE(6); t1 = GET_VALUE(7);
			val |= (t0 < t1) << 3;
			t0 = GET_VALUE(8); t1 = GET_VALUE(9);
			val |= (t0 < t1) << 4;
			t0 = GET_VALUE(10); t1 = GET_VALUE(11);
			val |= (t0 < t1) << 5;
			t0 = GET_VALUE(12); t1 = GET_VALUE(13);
			val |= (t0 < t1) << 6;
			t0 = GET_VALUE(14); t1 = GET_VALUE(15);
			val |= (t0 < t1) << 7;

			desc[i] = (uchar)val;
		}

#undef GET_VALUE
	}

//初始化用于计算描述子的pattern变量， pattern是用于计算描述子的256对坐标，其值是定死的
	static int bit_pattern_31_[256*4] =
		{
			8,-3, 9,5/*mean (0), correlation (0)*/,
			4,2, 7,-12/*mean (1.12461e-05), correlation (0.0437584)*/,
			-11,9, -8,2/*mean (3.37382e-05), correlation (0.0617409)*/,
			7,-12, 12,-13/*mean (5.62303e-05), correlation (0.0636977)*/,
			2,-13, 2,12/*mean (0.000134953), correlation (0.085099)*/,
			1,-7, 1,6/*mean (0.000528565), correlation (0.0857175)*/,
			-2,-10, -2,-4/*mean (0.0188821), correlation (0.0985774)*/,
			-13,-13, -11,-8/*mean (0.0363135), correlation (0.0899616)*/,
			-13,-3, -12,-9/*mean (0.121806), correlation (0.099849)*/,
			10,4, 11,9/*mean (0.122065), correlation (0.093285)*/,
			-13,-8, -8,-9/*mean (0.162787), correlation (0.0942748)*/,
			-11,7, -9,12/*mean (0.21561), correlation (0.0974438)*/,
			7,7, 12,6/*mean (0.160583), correlation (0.130064)*/,
			-4,-5, -3,0/*mean (0.228171), correlation (0.132998)*/,
			-13,2, -12,-3/*mean (0.00997526), correlation (0.145926)*/,
			-9,0, -7,5/*mean (0.198234), correlation (0.143636)*/,
			12,-6, 12,-1/*mean (0.0676226), correlation (0.16689)*/,
			-3,6, -2,12/*mean (0.166847), correlation (0.171682)*/,
			-6,-13, -4,-8/*mean (0.101215), correlation (0.179716)*/,
			11,-13, 12,-8/*mean (0.200641), correlation (0.192279)*/,
			4,7, 5,1/*mean (0.205106), correlation (0.186848)*/,
			5,-3, 10,-3/*mean (0.234908), correlation (0.192319)*/,
			3,-7, 6,12/*mean (0.0709964), correlation (0.210872)*/,
			-8,-7, -6,-2/*mean (0.0939834), correlation (0.212589)*/,
			-2,11, -1,-10/*mean (0.127778), correlation (0.20866)*/,
			-13,12, -8,10/*mean (0.14783), correlation (0.206356)*/,
			-7,3, -5,-3/*mean (0.182141), correlation (0.198942)*/,
			-4,2, -3,7/*mean (0.188237), correlation (0.21384)*/,
			-10,-12, -6,11/*mean (0.14865), correlation (0.23571)*/,
			5,-12, 6,-7/*mean (0.222312), correlation (0.23324)*/,
			5,-6, 7,-1/*mean (0.229082), correlation (0.23389)*/,
			1,0, 4,-5/*mean (0.241577), correlation (0.215286)*/,
			9,11, 11,-13/*mean (0.00338507), correlation (0.251373)*/,
			4,7, 4,12/*mean (0.131005), correlation (0.257622)*/,
			2,-1, 4,4/*mean (0.152755), correlation (0.255205)*/,
			-4,-12, -2,7/*mean (0.182771), correlation (0.244867)*/,
			-8,-5, -7,-10/*mean (0.186898), correlation (0.23901)*/,
			4,11, 9,12/*mean (0.226226), correlation (0.258255)*/,
			0,-8, 1,-13/*mean (0.0897886), correlation (0.274827)*/,
			-13,-2, -8,2/*mean (0.148774), correlation (0.28065)*/,
			-3,-2, -2,3/*mean (0.153048), correlation (0.283063)*/,
			-6,9, -4,-9/*mean (0.169523), correlation (0.278248)*/,
			8,12, 10,7/*mean (0.225337), correlation (0.282851)*/,
			0,9, 1,3/*mean (0.226687), correlation (0.278734)*/,
			7,-5, 11,-10/*mean (0.00693882), correlation (0.305161)*/,
			-13,-6, -11,0/*mean (0.0227283), correlation (0.300181)*/,
			10,7, 12,1/*mean (0.125517), correlation (0.31089)*/,
			-6,-3, -6,12/*mean (0.131748), correlation (0.312779)*/,
			10,-9, 12,-4/*mean (0.144827), correlation (0.292797)*/,
			-13,8, -8,-12/*mean (0.149202), correlation (0.308918)*/,
			-13,0, -8,-4/*mean (0.160909), correlation (0.310013)*/,
			3,3, 7,8/*mean (0.177755), correlation (0.309394)*/,
			5,7, 10,-7/*mean (0.212337), correlation (0.310315)*/,
			-1,7, 1,-12/*mean (0.214429), correlation (0.311933)*/,
			3,-10, 5,6/*mean (0.235807), correlation (0.313104)*/,
			2,-4, 3,-10/*mean (0.00494827), correlation (0.344948)*/,
			-13,0, -13,5/*mean (0.0549145), correlation (0.344675)*/,
			-13,-7, -12,12/*mean (0.103385), correlation (0.342715)*/,
			-13,3, -11,8/*mean (0.134222), correlation (0.322922)*/,
			-7,12, -4,7/*mean (0.153284), correlation (0.337061)*/,
			6,-10, 12,8/*mean (0.154881), correlation (0.329257)*/,
			-9,-1, -7,-6/*mean (0.200967), correlation (0.33312)*/,
			-2,-5, 0,12/*mean (0.201518), correlation (0.340635)*/,
			-12,5, -7,5/*mean (0.207805), correlation (0.335631)*/,
			3,-10, 8,-13/*mean (0.224438), correlation (0.34504)*/,
			-7,-7, -4,5/*mean (0.239361), correlation (0.338053)*/,
			-3,-2, -1,-7/*mean (0.240744), correlation (0.344322)*/,
			2,9, 5,-11/*mean (0.242949), correlation (0.34145)*/,
			-11,-13, -5,-13/*mean (0.244028), correlation (0.336861)*/,
			-1,6, 0,-1/*mean (0.247571), correlation (0.343684)*/,
			5,-3, 5,2/*mean (0.000697256), correlation (0.357265)*/,
			-4,-13, -4,12/*mean (0.00213675), correlation (0.373827)*/,
			-9,-6, -9,6/*mean (0.0126856), correlation (0.373938)*/,
			-12,-10, -8,-4/*mean (0.0152497), correlation (0.364237)*/,
			10,2, 12,-3/*mean (0.0299933), correlation (0.345292)*/,
			7,12, 12,12/*mean (0.0307242), correlation (0.366299)*/,
			-7,-13, -6,5/*mean (0.0534975), correlation (0.368357)*/,
			-4,9, -3,4/*mean (0.099865), correlation (0.372276)*/,
			7,-1, 12,2/*mean (0.117083), correlation (0.364529)*/,
			-7,6, -5,1/*mean (0.126125), correlation (0.369606)*/,
			-13,11, -12,5/*mean (0.130364), correlation (0.358502)*/,
			-3,7, -2,-6/*mean (0.131691), correlation (0.375531)*/,
			7,-8, 12,-7/*mean (0.160166), correlation (0.379508)*/,
			-13,-7, -11,-12/*mean (0.167848), correlation (0.353343)*/,
			1,-3, 12,12/*mean (0.183378), correlation (0.371916)*/,
			2,-6, 3,0/*mean (0.228711), correlation (0.371761)*/,
			-4,3, -2,-13/*mean (0.247211), correlation (0.364063)*/,
			-1,-13, 1,9/*mean (0.249325), correlation (0.378139)*/,
			7,1, 8,-6/*mean (0.000652272), correlation (0.411682)*/,
			1,-1, 3,12/*mean (0.00248538), correlation (0.392988)*/,
			9,1, 12,6/*mean (0.0206815), correlation (0.386106)*/,
			-1,-9, -1,3/*mean (0.0364485), correlation (0.410752)*/,
			-13,-13, -10,5/*mean (0.0376068), correlation (0.398374)*/,
			7,7, 10,12/*mean (0.0424202), correlation (0.405663)*/,
			12,-5, 12,9/*mean (0.0942645), correlation (0.410422)*/,
			6,3, 7,11/*mean (0.1074), correlation (0.413224)*/,
			5,-13, 6,10/*mean (0.109256), correlation (0.408646)*/,
			2,-12, 2,3/*mean (0.131691), correlation (0.416076)*/,
			3,8, 4,-6/*mean (0.165081), correlation (0.417569)*/,
			2,6, 12,-13/*mean (0.171874), correlation (0.408471)*/,
			9,-12, 10,3/*mean (0.175146), correlation (0.41296)*/,
			-8,4, -7,9/*mean (0.183682), correlation (0.402956)*/,
			-11,12, -4,-6/*mean (0.184672), correlation (0.416125)*/,
			1,12, 2,-8/*mean (0.191487), correlation (0.386696)*/,
			6,-9, 7,-4/*mean (0.192668), correlation (0.394771)*/,
			2,3, 3,-2/*mean (0.200157), correlation (0.408303)*/,
			6,3, 11,0/*mean (0.204588), correlation (0.411762)*/,
			3,-3, 8,-8/*mean (0.205904), correlation (0.416294)*/,
			7,8, 9,3/*mean (0.213237), correlation (0.409306)*/,
			-11,-5, -6,-4/*mean (0.243444), correlation (0.395069)*/,
			-10,11, -5,10/*mean (0.247672), correlation (0.413392)*/,
			-5,-8, -3,12/*mean (0.24774), correlation (0.411416)*/,
			-10,5, -9,0/*mean (0.00213675), correlation (0.454003)*/,
			8,-1, 12,-6/*mean (0.0293635), correlation (0.455368)*/,
			4,-6, 6,-11/*mean (0.0404971), correlation (0.457393)*/,
			-10,12, -8,7/*mean (0.0481107), correlation (0.448364)*/,
			4,-2, 6,7/*mean (0.050641), correlation (0.455019)*/,
			-2,0, -2,12/*mean (0.0525978), correlation (0.44338)*/,
			-5,-8, -5,2/*mean (0.0629667), correlation (0.457096)*/,
			7,-6, 10,12/*mean (0.0653846), correlation (0.445623)*/,
			-9,-13, -8,-8/*mean (0.0858749), correlation (0.449789)*/,
			-5,-13, -5,-2/*mean (0.122402), correlation (0.450201)*/,
			8,-8, 9,-13/*mean (0.125416), correlation (0.453224)*/,
			-9,-11, -9,0/*mean (0.130128), correlation (0.458724)*/,
			1,-8, 1,-2/*mean (0.132467), correlation (0.440133)*/,
			7,-4, 9,1/*mean (0.132692), correlation (0.454)*/,
			-2,1, -1,-4/*mean (0.135695), correlation (0.455739)*/,
			11,-6, 12,-11/*mean (0.142904), correlation (0.446114)*/,
			-12,-9, -6,4/*mean (0.146165), correlation (0.451473)*/,
			3,7, 7,12/*mean (0.147627), correlation (0.456643)*/,
			5,5, 10,8/*mean (0.152901), correlation (0.455036)*/,
			0,-4, 2,8/*mean (0.167083), correlation (0.459315)*/,
			-9,12, -5,-13/*mean (0.173234), correlation (0.454706)*/,
			0,7, 2,12/*mean (0.18312), correlation (0.433855)*/,
			-1,2, 1,7/*mean (0.185504), correlation (0.443838)*/,
			5,11, 7,-9/*mean (0.185706), correlation (0.451123)*/,
			3,5, 6,-8/*mean (0.188968), correlation (0.455808)*/,
			-13,-4, -8,9/*mean (0.191667), correlation (0.459128)*/,
			-5,9, -3,-3/*mean (0.193196), correlation (0.458364)*/,
			-4,-7, -3,-12/*mean (0.196536), correlation (0.455782)*/,
			6,5, 8,0/*mean (0.1972), correlation (0.450481)*/,
			-7,6, -6,12/*mean (0.199438), correlation (0.458156)*/,
			-13,6, -5,-2/*mean (0.211224), correlation (0.449548)*/,
			1,-10, 3,10/*mean (0.211718), correlation (0.440606)*/,
			4,1, 8,-4/*mean (0.213034), correlation (0.443177)*/,
			-2,-2, 2,-13/*mean (0.234334), correlation (0.455304)*/,
			2,-12, 12,12/*mean (0.235684), correlation (0.443436)*/,
			-2,-13, 0,-6/*mean (0.237674), correlation (0.452525)*/,
			4,1, 9,3/*mean (0.23962), correlation (0.444824)*/,
			-6,-10, -3,-5/*mean (0.248459), correlation (0.439621)*/,
			-3,-13, -1,1/*mean (0.249505), correlation (0.456666)*/,
			7,5, 12,-11/*mean (0.00119208), correlation (0.495466)*/,
			4,-2, 5,-7/*mean (0.00372245), correlation (0.484214)*/,
			-13,9, -9,-5/*mean (0.00741116), correlation (0.499854)*/,
			7,1, 8,6/*mean (0.0208952), correlation (0.499773)*/,
			7,-8, 7,6/*mean (0.0220085), correlation (0.501609)*/,
			-7,-4, -7,1/*mean (0.0233806), correlation (0.496568)*/,
			-8,11, -7,-8/*mean (0.0236505), correlation (0.489719)*/,
			-13,6, -12,-8/*mean (0.0268781), correlation (0.503487)*/,
			2,4, 3,9/*mean (0.0323324), correlation (0.501938)*/,
			10,-5, 12,3/*mean (0.0399235), correlation (0.494029)*/,
			-6,-5, -6,7/*mean (0.0420153), correlation (0.486579)*/,
			8,-3, 9,-8/*mean (0.0548021), correlation (0.484237)*/,
			2,-12, 2,8/*mean (0.0616622), correlation (0.496642)*/,
			-11,-2, -10,3/*mean (0.0627755), correlation (0.498563)*/,
			-12,-13, -7,-9/*mean (0.0829622), correlation (0.495491)*/,
			-11,0, -10,-5/*mean (0.0843342), correlation (0.487146)*/,
			5,-3, 11,8/*mean (0.0929937), correlation (0.502315)*/,
			-2,-13, -1,12/*mean (0.113327), correlation (0.48941)*/,
			-1,-8, 0,9/*mean (0.132119), correlation (0.467268)*/,
			-13,-11, -12,-5/*mean (0.136269), correlation (0.498771)*/,
			-10,-2, -10,11/*mean (0.142173), correlation (0.498714)*/,
			-3,9, -2,-13/*mean (0.144141), correlation (0.491973)*/,
			2,-3, 3,2/*mean (0.14892), correlation (0.500782)*/,
			-9,-13, -4,0/*mean (0.150371), correlation (0.498211)*/,
			-4,6, -3,-10/*mean (0.152159), correlation (0.495547)*/,
			-4,12, -2,-7/*mean (0.156152), correlation (0.496925)*/,
			-6,-11, -4,9/*mean (0.15749), correlation (0.499222)*/,
			6,-3, 6,11/*mean (0.159211), correlation (0.503821)*/,
			-13,11, -5,5/*mean (0.162427), correlation (0.501907)*/,
			11,11, 12,6/*mean (0.16652), correlation (0.497632)*/,
			7,-5, 12,-2/*mean (0.169141), correlation (0.484474)*/,
			-1,12, 0,7/*mean (0.169456), correlation (0.495339)*/,
			-4,-8, -3,-2/*mean (0.171457), correlation (0.487251)*/,
			-7,1, -6,7/*mean (0.175), correlation (0.500024)*/,
			-13,-12, -8,-13/*mean (0.175866), correlation (0.497523)*/,
			-7,-2, -6,-8/*mean (0.178273), correlation (0.501854)*/,
			-8,5, -6,-9/*mean (0.181107), correlation (0.494888)*/,
			-5,-1, -4,5/*mean (0.190227), correlation (0.482557)*/,
			-13,7, -8,10/*mean (0.196739), correlation (0.496503)*/,
			1,5, 5,-13/*mean (0.19973), correlation (0.499759)*/,
			1,0, 10,-13/*mean (0.204465), correlation (0.49873)*/,
			9,12, 10,-1/*mean (0.209334), correlation (0.49063)*/,
			5,-8, 10,-9/*mean (0.211134), correlation (0.503011)*/,
			-1,11, 1,-13/*mean (0.212), correlation (0.499414)*/,
			-9,-3, -6,2/*mean (0.212168), correlation (0.480739)*/,
			-1,-10, 1,12/*mean (0.212731), correlation (0.502523)*/,
			-13,1, -8,-10/*mean (0.21327), correlation (0.489786)*/,
			8,-11, 10,-6/*mean (0.214159), correlation (0.488246)*/,
			2,-13, 3,-6/*mean (0.216993), correlation (0.50287)*/,
			7,-13, 12,-9/*mean (0.223639), correlation (0.470502)*/,
			-10,-10, -5,-7/*mean (0.224089), correlation (0.500852)*/,
			-10,-8, -8,-13/*mean (0.228666), correlation (0.502629)*/,
			4,-6, 8,5/*mean (0.22906), correlation (0.498305)*/,
			3,12, 8,-13/*mean (0.233378), correlation (0.503825)*/,
			-4,2, -3,-3/*mean (0.234323), correlation (0.476692)*/,
			5,-13, 10,-12/*mean (0.236392), correlation (0.475462)*/,
			4,-13, 5,-1/*mean (0.236842), correlation (0.504132)*/,
			-9,9, -4,3/*mean (0.236977), correlation (0.497739)*/,
			0,3, 3,-9/*mean (0.24314), correlation (0.499398)*/,
			-12,1, -6,1/*mean (0.243297), correlation (0.489447)*/,
			3,2, 4,-8/*mean (0.00155196), correlation (0.553496)*/,
			-10,-10, -10,9/*mean (0.00239541), correlation (0.54297)*/,
			8,-13, 12,12/*mean (0.0034413), correlation (0.544361)*/,
			-8,-12, -6,-5/*mean (0.003565), correlation (0.551225)*/,
			2,2, 3,7/*mean (0.00835583), correlation (0.55285)*/,
			10,6, 11,-8/*mean (0.00885065), correlation (0.540913)*/,
			6,8, 8,-12/*mean (0.0101552), correlation (0.551085)*/,
			-7,10, -6,5/*mean (0.0102227), correlation (0.533635)*/,
			-3,-9, -3,9/*mean (0.0110211), correlation (0.543121)*/,
			-1,-13, -1,5/*mean (0.0113473), correlation (0.550173)*/,
			-3,-7, -3,4/*mean (0.0140913), correlation (0.554774)*/,
			-8,-2, -8,3/*mean (0.017049), correlation (0.55461)*/,
			4,2, 12,12/*mean (0.01778), correlation (0.546921)*/,
			2,-5, 3,11/*mean (0.0224022), correlation (0.549667)*/,
			6,-9, 11,-13/*mean (0.029161), correlation (0.546295)*/,
			3,-1, 7,12/*mean (0.0303081), correlation (0.548599)*/,
			11,-1, 12,4/*mean (0.0355151), correlation (0.523943)*/,
			-3,0, -3,6/*mean (0.0417904), correlation (0.543395)*/,
			4,-11, 4,12/*mean (0.0487292), correlation (0.542818)*/,
			2,-4, 2,1/*mean (0.0575124), correlation (0.554888)*/,
			-10,-6, -8,1/*mean (0.0594242), correlation (0.544026)*/,
			-13,7, -11,1/*mean (0.0597391), correlation (0.550524)*/,
			-13,12, -11,-13/*mean (0.0608974), correlation (0.55383)*/,
			6,0, 11,-13/*mean (0.065126), correlation (0.552006)*/,
			0,-1, 1,4/*mean (0.074224), correlation (0.546372)*/,
			-13,3, -9,-2/*mean (0.0808592), correlation (0.554875)*/,
			-9,8, -6,-3/*mean (0.0883378), correlation (0.551178)*/,
			-13,-6, -8,-2/*mean (0.0901035), correlation (0.548446)*/,
			5,-9, 8,10/*mean (0.0949843), correlation (0.554694)*/,
			2,7, 3,-9/*mean (0.0994152), correlation (0.550979)*/,
			-1,-6, -1,-1/*mean (0.10045), correlation (0.552714)*/,
			9,5, 11,-2/*mean (0.100686), correlation (0.552594)*/,
			11,-3, 12,-8/*mean (0.101091), correlation (0.532394)*/,
			3,0, 3,5/*mean (0.101147), correlation (0.525576)*/,
			-1,4, 0,10/*mean (0.105263), correlation (0.531498)*/,
			3,-6, 4,5/*mean (0.110785), correlation (0.540491)*/,
			-13,0, -10,5/*mean (0.112798), correlation (0.536582)*/,
			5,8, 12,11/*mean (0.114181), correlation (0.555793)*/,
			8,9, 9,-6/*mean (0.117431), correlation (0.553763)*/,
			7,-4, 8,-12/*mean (0.118522), correlation (0.553452)*/,
			-10,4, -10,9/*mean (0.12094), correlation (0.554785)*/,
			7,3, 12,4/*mean (0.122582), correlation (0.555825)*/,
			9,-7, 10,-2/*mean (0.124978), correlation (0.549846)*/,
			7,0, 12,-2/*mean (0.127002), correlation (0.537452)*/,
			-1,-6, 0,-11/*mean (0.127148), correlation (0.547401)*/
		};

//特征点提取器的构造函数
//在tracking线程调用，刚输入一帧图片的时候
	ORBExtractor::ORBExtractor(int _nfeatures,       //指定要提取的特征点数目
		float _scaleFactor,   //指定图像金字塔的缩放系数
		int _nlevels,        //指定图像金字塔的层数
		int _iniThFAST,        //指定初始的FAST特征点提取参数，可以提取出最明显的角点
		int _minThFAST):    //如果初始阈值没有检测到角点，降低到这个阈值提取出弱一点的角点
		mi_Features(_nfeatures), mf_scaleFactor(_scaleFactor), mi_Levels(_nlevels),
		mi_iniThFAST(_iniThFAST), mi_minThFAST(_minThFAST)
	{
		mv_scaleFactor.resize(mi_Levels);    //存储每层图像缩放系数的vector调整为符合图层数目的大小
		mv_levelSigma2.resize(mi_Levels);    //存储这个sigma^2，其实就是每层图像相对初始图像缩放因子的平方
		mv_scaleFactor[0]=1.0f;
		mv_levelSigma2[0]=1.0f;
		//然后逐层计算图像金字塔中图像相当于初始图像的缩放系数
		for(int i=1; i<mi_Levels; i++)
		{
			//其实就是这样累乘计算得出来的
			mv_scaleFactor[i]=mv_scaleFactor[i-1]*mf_scaleFactor;
			//原来这里的sigma^2就是每层图像相对于初始图像缩放因子的平方
			mv_levelSigma2[i]=mv_scaleFactor[i]*mv_scaleFactor[i];
		}
		//接下来的两个向量保存上面的参数的倒数
		mv_invScaleFactor.resize(mi_Levels);
		mv_invLevelSigma2.resize(mi_Levels);
		for(int i=0; i<mi_Levels; i++)
		{
			mv_invScaleFactor[i]=1.0f/mv_scaleFactor[i];
			mv_invLevelSigma2[i]=1.0f/mv_levelSigma2[i];
		}
		//调整图像金字塔vector以使得其符合设定的图像层数
		mv_imagePyramid.resize(mi_Levels);

		//每层需要提取出来的特征点个数，这个向量也要根据图像金字塔设定的层数进行调整
		mv_featuresPerLevel.resize(mi_Levels);
		//图片降采样缩放系数的倒数
		float factor = 1.0f / mf_scaleFactor;
		//第0层图像应该分配的特征点数量这里的值  1000*(1-0.2/1.2)/(1-pow(0.2/1.2, 8)) = 216
		// {61， 73， 87， 105， 126， 151， 181， 216}
		float nDesiredFeaturesPerScale = mi_Features*(1 - factor)/(1 - (float)pow((double)factor, (double)mi_Levels));

		int sumFeatures = 0;
		// 计算每层特征点的数量
		for( int level = 0; level < mi_Levels-1; level++ )
		{
			mv_featuresPerLevel[level] = cvRound(nDesiredFeaturesPerScale);
			sumFeatures += mv_featuresPerLevel[level];
			nDesiredFeaturesPerScale *= factor;
		}
		//由于前面的特征点个数取整操作，可能会导致剩余一些特征点个数没有被分配，所以这里就将这个余出来的特征点分配到最高的图层中
		mv_featuresPerLevel[mi_Levels-1] = std::max(mi_Features - sumFeatures, 0);

		//成员变量pattern的长度，也就是点的个数，这里的512表示512个点（上面的数组中是存储的坐标所以是256*2*2）
		const int npoints = 512;
		//获取用于计算BRIEF描述子的随机采样点点集头指针
		//注意到pattern0数据类型为Points*,bit_pattern_31_是int[]型，所以这里需要进行强制类型转换
		const cv::Point* pattern0 = (const cv::Point*)bit_pattern_31_;

		//使用std::back_inserter的目的是可以快覆盖掉这个容器pattern之前的数据
		//其实这里的操作就是，将在全局变量区域的、int格式的随机采样点以cv::point格式复制到当前类对象中的成员变量中
		std::copy(pattern0, pattern0 + npoints, std::back_inserter(pattern));

		//This is for orientation
		//下面的内容是和特征点的旋转计算有关的
		// pre-compute the end of a row in a circular patch
		//预先计算圆形patch中行的结束位置
		//+1中的1表示那个圆的中间行
		mv_Max.resize(HALF_PATCH_SIZE + 1);
		//cvFloor返回不大于参数的最大整数值，cvCeil返回不小于参数的最小整数值，cvRound则是四舍五入
		int v,  //循环辅助变量
		v0,     //辅助变量
		vmax = cvFloor(HALF_PATCH_SIZE * sqrt(2.f) / 2 + 1); //45度射线与圆周坐标焦点的纵坐标 此处结果为11
		int vmin = cvCeil(HALF_PATCH_SIZE * sqrt(2.f) / 2);//45度射线与圆周坐标焦点的纵坐标 此处结果为11
		const double hp2 = HALF_PATCH_SIZE*HALF_PATCH_SIZE;//半径的平方

		//半径16圆的横坐标为u，纵坐标为v
		//这里先计算第一象限下半部分45度圆的u取值的最大，结果是15，14，14，14，14，14，13，13，12，12，11，10
		for (v = 0; v <= vmax; ++v)
			mv_Max[v] = cvRound(sqrt(hp2 - v * v));

		// Make sure we are symmetric
		// 根据对称性补充第一象限上半部分45度圆的u取值的最大，通过计算重复的个数得到，结果是3，6，8，9
		for (v = HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v)
		{
			while (mv_Max[v0] == mv_Max[v0 + 1])
				++v0;
			mv_Max[v] = v0;
			++v0;
		}
	}

	static void computeOrientation(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const std::vector<int>& umax)
	{
		//遍历所有的特征点
		for (std::vector<cv::KeyPoint>::iterator keypoint = keypoints.begin(),
			     keypointEnd = keypoints.end(); keypoint != keypointEnd; ++keypoint)
		{
			//调用IC_Angle函数计算这个特征点的大小重心方向
			keypoint->angle = IC_Angle(image, //特征点所在当前金字塔的图像
				keypoint->pt,    //特征点在这张途中的坐标
				umax); //每个特征点所在图像区块的每行的边界 u_max 组成的vector(四分之圆)
		}
	}

	void ExtractorNode::DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4)
	{
		//得到当前提取器节点所在图像区域的一半长宽，当然结果需要取整
		const int halfX = ceil(static_cast<float>(UR.x-UL.x)/2);
		const int halfY = ceil(static_cast<float>(BR.y-UL.y)/2);

		//Define boundaries of childs
		//下面的操作大同小异，将一个图像区域再细分成为四个小图像区块
		//n1 存储左上区域的边界
		n1.UL = UL;
		n1.UR = cv::Point2i(UL.x+halfX,UL.y);
		n1.BL = cv::Point2i(UL.x,UL.y+halfY);
		n1.BR = cv::Point2i(UL.x+halfX,UL.y+halfY);
		//用来存储在该节点对应的图像网格中提取出来的特征点的vector
		n1.mv_Keys.reserve(mv_Keys.size());

		n2.UL = n1.UR;
		n2.UR = UR;
		n2.BL = n1.BR;
		n2.BR = cv::Point2i(UR.x,UL.y+halfY);
		n2.mv_Keys.reserve(mv_Keys.size());

		n3.UL = n1.BL;
		n3.UR = n1.BR;
		n3.BL = BL;
		n3.BR = cv::Point2i(n1.BR.x,BL.y);
		n3.mv_Keys.reserve(mv_Keys.size());

		n4.UL = n3.UR;
		n4.UR = n2.BR;
		n4.BL = n3.BR;
		n4.BR = BR;
		n4.mv_Keys.reserve(mv_Keys.size());

		//Associate points to childs
		//遍历当前提取器节点的vkeys中存储的特征点
		for(size_t i=0;i<mv_Keys.size();i++)
		{
			//获取这个特征点对象
			const cv::KeyPoint &kp = mv_Keys[i];
			//判断这个特征点在当前特征点提取器节点图像的哪个区域，更严格地说是属于那个子图像区块
			//然后就将这个特征点追加到那个特征点提取器节点的vkeys中
			//NOTICE BUG REVIEW 这里也是直接进行比较的，但是特征点的坐标是在“半径扩充图像”坐标系下的，而节点区域的坐标则是在“边缘扩充图像”坐标系下的
			if(kp.pt.x<n1.UR.x)
			{
				if(kp.pt.y<n1.BR.y)
					n1.mv_Keys.push_back(kp);
				else
					n3.mv_Keys.push_back(kp);
			}
			else if(kp.pt.y<n1.BR.y)
				n2.mv_Keys.push_back(kp);
			else
				n4.mv_Keys.push_back(kp);
		}
		//判断每个子特征点提取器节点所在的图像中特征点的数目（就是分配给子节点的特征点数目），然后做标记
		// 这里判断是否数目等于1的目的是确定这个节点还能不能再向下进行分裂
		if(n1.mv_Keys.size()==1)
			n1.mb_NoMore = true;
		if(n2.mv_Keys.size()==1)
			n2.mb_NoMore = true;
		if(n3.mv_Keys.size()==1)
			n3.mb_NoMore = true;
		if(n4.mv_Keys.size()==1)
			n4.mb_NoMore = true;

	}

	std::vector<cv::KeyPoint> ORBExtractor::DistributeQuadTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
		const int &maxX, const int &minY, const int &maxY, const int &N, const int &level)
	{
		// Compute how many initial nodes
		// Step 1 根据宽高比确定初始节点数目
		//计算应该生成的初始节点个数，根节点的数量nIni是根据边界的宽高比值确定的，一般是1或者2
		// ! bug: 如果宽高比小于0.5，nIni=0, 后面hx会报错
		// 例如maxX = 300， minX = 0， maxY = 100， minY = 0
		// 则nIni = 3
		// 主要目的是防止图像过长，或者过宽，如果过长或者过宽的化就多形成几个初始节点
		const int nIni = round(static_cast<float>(maxX-minX)/(maxY-minY));

		//一个初始的节点的x方向有多少个像素
		//hx = 100，则横向分为3部分, 每一部分100个像素点横向
		const float hX = static_cast<float>(maxX-minX)/nIni;

		//存储有提取器节点的链表
		std::list<ExtractorNode> lNodes;

		//存储初始提取器节点指针的vector
		std::vector<ExtractorNode*> vpIniNodes;
		//重新设置其大小
		vpIniNodes.resize(nIni);

		// Step 2 生成初始提取器节点
		for(int i=0; i<nIni; i++)
		{   //生成一个提取器节点
			ExtractorNode ni;
			//设置提取器节点的图像边界
			//注意这里和提取FAST角点区域相同，都是“半径扩充图像”，特征点坐标从0 开始
			ni.UL = cv::Point2i(hX*static_cast<float>(i),0);
			ni.UR = cv::Point2i(hX*static_cast<float>(i+1),0);
			ni.BL = cv::Point2i(ni.UL.x,maxY-minY);
			ni.BR = cv::Point2i(ni.UR.x,maxY-minY);
			//重设vkeys大小
			ni.mv_Keys.reserve(vToDistributeKeys.size());
			//将刚才生成的提取节点添加到链表中
			//虽然这里的ni是局部变量，但是由于这里的push_back()是拷贝参数的内容到一个新的对象中然后再添加到列表中
			//所以当本函数退出之后这里的内存不会成为“野指针”
			lNodes.push_back(ni);
			//存储这个初始的提取器节点句柄
			vpIniNodes[i] = &lNodes.back();
		}

		//Associate points to childs
		// Step 3 将特征点分配到子提取器节点中
		for(size_t i=0;i<vToDistributeKeys.size();i++)
		{
			//获取这个特征点对象
			const cv::KeyPoint &kp = vToDistributeKeys[i];
			//按特征点的横轴位置，分配给属于那个图像区域的提取器节点（最初的提取器节点）

			//如果kp.pt.x/hx是小数，则向下取整
			vpIniNodes[kp.pt.x/hX]->mv_Keys.push_back(kp);
		}
		// 遍历此提取器节点列表，标记那些不可再分裂的节点，删除那些没有分配到特征点的节点
		// ? 这个步骤是必要的吗？感觉可以省略，通过判断nIni个数和vKeys.size() 就可以吧
		std::list<ExtractorNode>::iterator lit = lNodes.begin();

		while(lit!=lNodes.end())
		{
			//如果初始的提取器节点所分配到的特征点个数为1
			if(lit->mv_Keys.size()==1)
			{
				//那么就标志位复位，表示此节点不可再分
				lit->mb_NoMore=true;
				//更新迭代器
				lit++;
			}
				//如果一个提取器节点没有被分配到特征点，那么就从列表中直接删除它
			else if(lit->mv_Keys.empty())
				//由于是直接删除掉，所以这里的迭代器没有必要更新，否则反而会造成跳过元素的情况
				lit = lNodes.erase(lit);
			else
				//如果上面的这些情况和当前的特征点提取器节点无关，那么就只是更新迭代器
				lit++;
		}

		//结束标志位清空
		bool bFinish = false;

		//记录迭代次数，只是记录，并未起到作用
		int iteration = 0;

		//声明一个vector用于存储节点的vSize和句柄对
		//这个变量记录了在一次分裂循环中，那些可以再继续进行分裂的节点中包含的特征点数目和其句柄
		std::vector<std::pair<int,ExtractorNode*> > vSizeAndPointerToNode;
		//调整大小，这里的意思是一个初始化节点将分裂成为四个，当然实际上不一定有这么多，这里多分配了一些只是预防万一
		vSizeAndPointerToNode.reserve(lNodes.size()*4);

		//Step 5,根据特征点分布，利用四叉树的方法对图像进行划分区域
		while(!bFinish)
		{
			//更新迭代次数计数器，只是记录，并未起到作用
			iteration++;

			//保存当前节点个数，prev在这理解为“保留”
			int prevSize = lNodes.size();

			//重新定位迭代器指向列表头部
			lit = lNodes.begin();

			//需要展开的节点计数，这个一直保持累加，不清零
			int nToExpand = 0;

			//因为是在循环中，前面的循环体中可能污染了这个变量，所以清空
			//这个变量也只是统计了某一个循环中的点
			//这个变量记录了在一次分裂循环中，那些可以再继续进行分裂的节点中包含的特征点数目和其句柄
			vSizeAndPointerToNode.clear();


			while(lit!=lNodes.end())
			{
				//如果该节点的特征点只有一个，则该节点不能再分裂了，就跳过
				if(lit->mb_NoMore)
				{
					// If node only contains one point do not subdivide and continue
					lit++;
					continue;
				}
				else
				{
					// If more than one point, subdivide
					//如果该节点具有超过一个以上的特征点，那就继续分
					ExtractorNode n1,n2,n3,n4;
					//再划分成4个节点
					lit->DivideNode(n1,n2,n3,n4);

					// Add childs if they contain points
					//如果这里分出来的子区域中有特征点，那么就将这个子区域的节点添加到提取器节点的报表中
					//注意这里的条件是，有特征点即可
					if(n1.mv_Keys.size()>0)
					{
						//注意这里是添加到列表最前端
						lNodes.push_front(n1);
						//再判断其中的子提取器的节点的特征点数量是否大于1
						if(n1.mv_Keys.size()>1)
						{
							// 如果有超过一个的特征点，那么需要展开的节点计数就+1
							nToExpand++;
							//保存这个特征点数目和节点指针的信息
							vSizeAndPointerToNode.push_back(std::make_pair(n1.mv_Keys.size(),&lNodes.front()));
							// lNodes.front().lit 和前面的迭代的lit 不同，只是名字相同而已
							// lNodes.front().lit是node结构体里的一个指针用来记录节点的位置
							// 迭代的lit 是while循环里作者命名的遍历的指针名称
							lNodes.front().lit = lNodes.begin();
						}
					}
					if(n2.mv_Keys.size()>0)
					{
						lNodes.push_front(n2);
						if(n2.mv_Keys.size()>1)
						{
							nToExpand++;
							vSizeAndPointerToNode.push_back(std::make_pair(n2.mv_Keys.size(),&lNodes.front()));
							lNodes.front().lit = lNodes.begin();
						}
					}
					if(n3.mv_Keys.size()>0)
					{
						lNodes.push_front(n3);
						if(n3.mv_Keys.size()>1)
						{
							nToExpand++;
							vSizeAndPointerToNode.push_back(std::make_pair(n3.mv_Keys.size(),&lNodes.front()));
							lNodes.front().lit = lNodes.begin();
						}
					}
					if(n4.mv_Keys.size()>0)
					{
						lNodes.push_front(n4);
						if(n4.mv_Keys.size()>1)
						{
							nToExpand++;
							vSizeAndPointerToNode.push_back(std::make_pair(n4.mv_Keys.size(),&lNodes.front()));
							lNodes.front().lit = lNodes.begin();
						}
					}
					//当这个母节点expand之后就从列表中删除它了，能够进行分裂操作说明至少一个子节点的区域中特征点的数量是大于1的
					//
					lit=lNodes.erase(lit);
					continue;
				}
			}

			// Finish if there are more nodes than required features
			// or all nodes contain just one point
			//停止迭代的两个条件，满足其中一个即可
			//1. 当前节点已经超过了要求的特征点数
			//2. 当前所有节点中都只包含一个特征点
			if((int)lNodes.size()>=N        //判断是否超过了要求的特征点数，这里因为一个节点至少一个特征点
				|| (int)lNodes.size()==prevSize) //prevSize保存的是分裂之前的节点个数， 如果分裂之前和分裂之后的总节点个数一样，则说明当前节点区域中每个只有一个特征点了，不能再分了
			{
				//停止分裂的标志位
				bFinish = true;
			}

				// Step 6 当再划分之后所有的Node数大于要求数目时,就慢慢划分直到使其刚刚达到或者超过要求的特征点个数
				//可以展开的子节点个数nToExpand x3，是因为一分四之后，会删除原来的主节点，所以乘以3（也可以理解为前面是加的lNodes是自身）

			else if(((int)lNodes.size()+nToExpand*3)>N)
			{
				//如果再分裂一次那么数目就要超了，这里想办法尽可能使其刚刚达到或者超过要求的特征点个数时就退出
				//这里的nToExpand和vSizeAndPointerToNode不是一次循环对一次循环的关系，而是前者是累计计数，后者只保存某一个循环的
				//一直循环，直到结束标志位被置位
				while(!bFinish)
				{

					prevSize = lNodes.size();
					//保留那些还可以分裂的节点的信息, 这里是深拷贝
					std::vector<std::pair<int,ExtractorNode*> > vPrevSizeAndPointerToNode = vSizeAndPointerToNode;
					vSizeAndPointerToNode.clear();
					// 对需要划分的节点进行排序，对pair对的第一个元素进行排序，默认是从小到大排序
					// 优先分裂特征点多的节点，使得特征点密集的区域保留更少的特征点
					//! 注意这里的排序规则非常重要！会导致每次最后产生的特征点都不一样。建议使用 stable_sort
					sort(vPrevSizeAndPointerToNode.begin(),vPrevSizeAndPointerToNode.end());

					//从后往前开始分，也就是从特征点多的开始分
					for(int j=vPrevSizeAndPointerToNode.size()-1;j>=0;j--)
					{
						ExtractorNode n1,n2,n3,n4;
						vPrevSizeAndPointerToNode[j].second->DivideNode(n1,n2,n3,n4);

						// Add childs if they contain points
						if(n1.mv_Keys.size()>0)
						{
							lNodes.push_front(n1);
							if(n1.mv_Keys.size()>1)
							{
								vSizeAndPointerToNode.push_back(std::make_pair(n1.mv_Keys.size(),&lNodes.front()));
								lNodes.front().lit = lNodes.begin();
							}
						}
						if(n2.mv_Keys.size()>0)
						{
							lNodes.push_front(n2);
							if(n2.mv_Keys.size()>1)
							{
								vSizeAndPointerToNode.push_back(std::make_pair(n2.mv_Keys.size(),&lNodes.front()));
								lNodes.front().lit = lNodes.begin();
							}
						}
						if(n3.mv_Keys.size()>0)
						{
							lNodes.push_front(n3);
							if(n3.mv_Keys.size()>1)
							{
								vSizeAndPointerToNode.push_back(std::make_pair(n3.mv_Keys.size(),&lNodes.front()));
								lNodes.front().lit = lNodes.begin();
							}
						}
						if(n4.mv_Keys.size()>0)
						{
							lNodes.push_front(n4);
							if(n4.mv_Keys.size()>1)
							{
								vSizeAndPointerToNode.push_back(std::make_pair(n4.mv_Keys.size(),&lNodes.front()));
								lNodes.front().lit = lNodes.begin();
							}
						}

						lNodes.erase(vPrevSizeAndPointerToNode[j].second->lit);
						//判断是是否超过了需要的特征点数？是的话就退出，不是的话就继续这个分裂过程，直到刚刚达到或者超过要求的特征点个数
						//作者的思想其实就是这样的，再分裂了一次之后判断下一次分裂是否会超过N，如果不是那么就放心大胆地全部进行分裂（因为少了一个判断因此
						//其运算速度会稍微快一些），如果会那么就引导到这里进行最后一次分裂
						if((int)lNodes.size()>=N)
							break;
					}
					//判断是否达到了停止条件
					if((int)lNodes.size()>=N || (int)lNodes.size()==prevSize)
						bFinish = true;

				}
			}
		}

		// Retain the best point in each node
		// Step 7 保留每个区域响应值最大的一个兴趣点
		//使用这个vector来存储我们感兴趣的特征点的过滤结果
		std::vector<cv::KeyPoint> vResultKeys;
		vResultKeys.reserve(mi_Features);
		//遍历每个节点的链表
		for(std::list<ExtractorNode>::iterator lit=lNodes.begin(); lit!=lNodes.end(); lit++)
		{
			std::vector<cv::KeyPoint> &vNodeKeys = lit->mv_Keys;
			//得到指向第一个特征点的指针，后面作为最大响应值对应的关键点
			cv::KeyPoint* pKP = &vNodeKeys[0];
			//用第1个关键点响应值初始化最大响应值
			float maxResponse = pKP->response;

			for(size_t k=1;k<vNodeKeys.size();k++)
			{
				if(vNodeKeys[k].response>maxResponse)
				{
					pKP = &vNodeKeys[k];
					maxResponse = vNodeKeys[k].response;
				}
			}
			//将这个节点区域中的响应值最大的特征点加入最终结果容器
			vResultKeys.push_back(*pKP);
		}
		//返回最终结果容器，其中保存有分裂出来的区域中，我们最感兴趣、响应值最大的特征点
		return vResultKeys;
	}
/*
 * 采用两种方法来平均特征点， 1是采用不同的阀值，2是使用四叉树，舍弃多余的特征点
 */
//计算四叉树的特征点，函数名字后面的OctTree只是说明了在过滤和分配特征点时所使用的方式
	void ORBExtractor::ComputeKeyPointsQuadTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints) //  所有特征点，这里第一层vector存储的是某图层
//第二层存储的该图层里面的所有特征点
	{
		//重新调整图像层数
		allKeypoints.resize(mi_Levels);
		//图像cell的尺寸，是个正方形，可以理解为边长in图像坐标
		const float W = 30;
		// 对每一层图像做处理
		// 遍历所有图像
		for (int level = 0; level < mi_Levels; ++level)
		{
			//计算图像边缘
			//计算这层图像的坐标边界， NOTICE 注意这里是坐标边界，EDGE_THRESHOLD指的应该是可以提取特征点的有效图像边界，后面会一直使用“有效图像边界“这个自创名词
			//FAST提取特征点有效半径是3
			const int minBorderX = EDGE_THRESHOLD-3;
			const int minBorderY = minBorderX;
			const int maxBorderX = mv_imagePyramid[level].cols-EDGE_THRESHOLD+3;
			const int maxBorderY = mv_imagePyramid[level].rows-EDGE_THRESHOLD+3;

			//存储需要进行平均分配的特征点
			std::vector<cv::KeyPoint> vToDistributeKeys;
			//一般都是过量采集，所以这里预分配的空间大小是nfeatures*10
			vToDistributeKeys.reserve(mi_Features*10);
			//reserve设置vector中的capacity()的值
			//vector中的capacity的值是指vector允许放入多少元素
			//vector中的size是指vector实际有多少元素

			//计算进行特征提取的图像区域尺寸
			const float width = (maxBorderX-minBorderX);
			const float height = (maxBorderY-minBorderY);

			const int nCols = width/W;//列
			const int nRows = height/W;//行
			const int wCell = ceil(width/nCols);
			const int hCell = ceil(height/nRows);
			// 每一行有多少cell
			// 每一列有多少cell
			// 每个cell的宽度向上取整
			// 每个cell的高度

			//1.遍历每一行和每一列，依次分别用高低阀值搜素FAST特征点
			//不理解为什么行列不用统一标准，行(即下面，即分行的时候剩3行不把他分为一个cell)剩3个跳过，列(即右面)剩6个跳过
			for(int i=0; i<nRows; i++)
			{
				//计算当前网络格的初始行坐标
				const float iniY =minBorderY+i*hCell;
				//计算当前网格的最大值，这里的6表示3+3，即考虑到了多出来以便进行FAST特征点提取用的3像素边界
				float maxY = iniY+hCell+6;

				//如果初始坐标就已经超过了有效的图像边界了，这里的有效边界是指可以提取FAST特征点的图像区域
				if(iniY>=maxBorderY-3)
					continue;
				if(maxY>maxBorderY)
					maxY = maxBorderY;

				//开始行的遍历
				for(int j=0; j<nCols; j++)
				{
					//计算当前网络格的初始列坐标
					const float iniX =minBorderX+j*wCell;
					float maxX = iniX+wCell+6;
					if(iniX>=maxBorderX-6)
						continue;
					if(maxX>maxBorderX)
						maxX = maxBorderX;

					std::vector<cv::KeyPoint> vKeysCell;
					//先用高阀值搜索，如果搜索不到的话，再用底阀值搜索
					FAST(mv_imagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),//待检测的图像，这里就是遍历到的图像块
						vKeysCell,//储存角点位置的容器
						mi_iniThFAST,//检测阀值
						true);//使能非极大抑制
					//如果这个图像块中使用默认的参数未能检测出特征点，就降低标准
					if(vKeysCell.empty())
					{
						FAST(mv_imagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),//待检测的图像，这里就是遍历到的图像块
							vKeysCell,//储存角点位置的容器
							mi_minThFAST,//更低的检测阀值
							true);//使能非极大抑制
					}

					//将vkeyscell的值添加到容器vToDistributeKeys中
					if(!vKeysCell.empty())
					{
						//遍历其中所有的FAST角点
						for(std::vector<cv::KeyPoint>::iterator vit=vKeysCell.begin(); vit!=vKeysCell.end();vit++)
						{
							//把网格里面特征点的坐标转换为全图坐标
							(*vit).pt.x+=j*wCell;
							(*vit).pt.y+=i*hCell;
							vToDistributeKeys.push_back(*vit);
						}
					}
				}
			}

			//声明一个对当前图层的特征点的容器的引用
			std::vector<cv::KeyPoint> & keypoints = allKeypoints[level];
			//并且调整其大小为欲提取出来的特征点个数
			keypoints.reserve(mi_Features);

			//对提取的 特征值进行八叉树筛选
			keypoints = DistributeQuadTree(vToDistributeKeys, //当前图层提取出来的特征点，也即是等待剔除的特征点，NOTICE注意此时刻特征点所使用的坐标都是在半径扩充图像下的
				minBorderX, maxBorderX, //当前图层的边界
				minBorderY, maxBorderY,
				mv_featuresPerLevel[level], //希望保留下来的当前层图像的特征点个数
				level);   //当前层图像所在的图层

			const int scaledPatchSize = PATCH_SIZE*mv_scaleFactor[level];

			// Add border to coordinates and scale information
			const int nkps = keypoints.size();
			//然后开始遍历这些特征点，恢复其在当前图层图像坐标系下的坐标
			for(int i=0; i<nkps ; i++)
			{
				//对每一个保留下来的特征点，恢复到相对于当前图层“边缘扩充图像下”的坐标系的坐标
				keypoints[i].pt.x+=minBorderX;
				keypoints[i].pt.y+=minBorderY;
				//记录特征点来源的图像金字塔图层
				keypoints[i].octave=level;
				keypoints[i].size = scaledPatchSize;
			}
		}

		// compute orientations
		//计算每个特征值的方向
		for (int level = 0; level < mi_Levels; ++level)
			computeOrientation(mv_imagePyramid[level], allKeypoints[level], mv_Max);
	}


	static void computeDescriptors(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors,
		const std::vector<cv::Point>& pattern)
	{
		descriptors = cv::Mat::zeros((int)keypoints.size(), 32, CV_8UC1);

		for (size_t i = 0; i < keypoints.size(); i++)
			computeOrbDescriptor(keypoints[i], image, &pattern[0], descriptors.ptr((int)i));
	}

	void ORBExtractor::operator()( cv::InputArray _image, std::vector<cv::KeyPoint>& _keypoints,
		cv::OutputArray _descriptors)
	{
		//检查图像的有效性
		if(_image.empty())
			return;
		// 获取图像的大小
		cv::Mat image = _image.getMat();
		//判断图像的格式是否正确，要求是单通道灰度值
		assert(image.type() == CV_8UC1 );

		// Pre-compute the scale pyramid
		//构建图像金字塔
		ComputePyramid(image);

		//计算特征点并进行四叉树筛选
		// 存储所有的特征点，注意此处为二维的vector，第一维存储的是金字塔的层数，第二维存储的是那一层金字塔图像里提取的所有特征点
		std::vector < std::vector<cv::KeyPoint> > allKeypoints;
		//使用四叉树的方式计算每层图像的特征点并进行分配
		ComputeKeyPointsQuadTree(allKeypoints);
		//ComputeKeyPointsOld(allKeypoints);

		// 拷贝图像描述子到新的矩阵descriptors
		cv::Mat descriptors;

		//统计整个图像金字塔中的特征点
		int nkeypoints = 0;
		for (int level = 0; level < mi_Levels; ++level)
			nkeypoints += (int)allKeypoints[level].size();
		//如果本图像金字塔中没有任何的特征点
		if( nkeypoints == 0 )
			//通过调用cv::mat类的.realse方法，强制清空矩阵的引用计数，这样就可以强制释放矩阵的数据了
			//参考[https://blog.csdn.net/giantchen547792075/article/details/9107877]
			_descriptors.release();
		else
		{   //如果图像金字塔中有特征点，那么就创建这个存储描述子的矩阵，注意这个矩阵是存储整个图像金字塔中特征点的描述子的
			_descriptors.create(nkeypoints,//矩阵的行数，对应为特征点的总个数
				32, //矩阵的列数，对应为使用32*8=256位描述子
				CV_8U); //矩阵元素的格式
			//获取这个描述子的矩阵信息
			//?为什么不是直接在参数_descriptors上对矩阵内容进行修改，而是重新新建了一个变量，复制矩阵后，在这个新建变量的基础上进行修改？
			descriptors = _descriptors.getMat();
		}
		//清空用作返回特征点提取结果的vector容器
		_keypoints.clear();
		_keypoints.reserve(nkeypoints);

		//遍历每一层图像，计算描述子
		//因为遍历是一层一层进行的，但是描述子那个矩阵是存储整个图像金字塔中特征点的描述子，所以在这里设置了Offset变量来保存“寻址”时的偏移量，
		//辅助进行在总描述子mat中的定位
		int offset = 0;
		//开始遍历每一层图像
		for (int level = 0; level < mi_Levels; ++level)
		{
			//获取在allKeypoints中当前层特征点容器的句柄
			std::vector<cv::KeyPoint>& keypoints = allKeypoints[level];
			//本层的特征点数
			int nkeypointsLevel = (int)keypoints.size();

			if(nkeypointsLevel==0)
				continue;

			// preprocess the resized image
			//  对图像进行高斯模糊
			// 深拷贝当前金字塔所在层级的图像
			cv::Mat workingMat = mv_imagePyramid[level].clone();
			//计算描述子之前进行一次高斯模糊
			GaussianBlur(workingMat, workingMat, cv::Size(7, 7), 2, 2, cv::BORDER_REFLECT_101);

			// Compute the descriptors
			// 计算高斯模糊后图像的描述子
			// desc存储当前图层的描述子
			// 为什么不分成一个二维vector？要把所有的特征点放在一个Mat里
			cv::Mat desc = descriptors.rowRange(offset, offset + nkeypointsLevel);
			computeDescriptors(workingMat, keypoints, desc, pattern);

			// 更新偏移量的值
			offset += nkeypointsLevel;


			if (level != 0)
			{
				// 获取当前图层上的缩放系数
				float scale = mv_scaleFactor[level]; //getScale(level, firstLevel, scaleFactor);
				for (std::vector<cv::KeyPoint>::iterator keypoint = keypoints.begin(),
					     keypointEnd = keypoints.end(); keypoint != keypointEnd; ++keypoint)
					// 特征点本身直接乘缩放倍数就可以了
					keypoint->pt *= scale;
			}

			_keypoints.insert(_keypoints.end(), keypoints.begin(), keypoints.end());
		}
	}

	void ORBExtractor::ComputePyramid(cv::Mat image)
	{
		//开始遍历所有的图层
		for (int level = 0; level < mi_Levels; ++level)
		{
			//计算缩放+补padding后该层图像的尺寸
			float scale = mv_invScaleFactor[level];
			cv::Size sz(cvRound((float)image.cols*scale), cvRound((float)image.rows*scale));
			//EDGE_THRESHOLD = 19;
			//即为周围填充的区域，上下左右各填充19，等于u方向填充38，v方向填充38
			cv::Size wholeSize(sz.width + EDGE_THRESHOLD*2, sz.height + EDGE_THRESHOLD*2);
			cv::Mat temp(wholeSize, image.type()),  masktemp;

			mv_imagePyramid[level] = temp(cv::Rect(EDGE_THRESHOLD, EDGE_THRESHOLD, sz.width, sz.height));

			if( level != 0 )
			{
				//把原图缩放成sz大小，同时改变的还有内容
				resize(mv_imagePyramid[level-1],//输入图像
					mv_imagePyramid[level],//输出图像
					sz,//输出图像的尺寸
					0,//水平方向上的缩放系数, 留0表示自动计算
					0,// 垂直方向上的缩放系数， 留0表示自动计算
					cv::INTER_LINEAR);

				copyMakeBorder(mv_imagePyramid[level], //输入图像
					temp, //输出图像
					EDGE_THRESHOLD, EDGE_THRESHOLD,//top&bottom  需要扩展的border大小
					EDGE_THRESHOLD, EDGE_THRESHOLD,//left&right 需要扩展的border大小
					cv::BORDER_REFLECT_101+cv::BORDER_ISOLATED);//扩充方式
			}
			else
			{
				copyMakeBorder(image, temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
					cv::BORDER_REFLECT_101);
			}
			//copymakeborder()函数同时进行缩放的同事进行填充；
		}

	}
}