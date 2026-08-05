#include ".\PSINSCore\kfapp.h"
#include ".\Demo\PSINS_Demo.h"

void main(void)
{
	CFileRdWt::Dir("D:\\ygm2026\\精准\\陀螺星敏\\");
	CFileRdWt fins("res.bin");
	CFileRdWt fimu("wqt.bin",-8);
	CGyroCNS gyroCns(TS10);
	for(int i=0; i<100*300; i++)
	{
		if(!fimu.load(1)) break;  // 读取传感器数据
		CVect3 wm=*(CVect3*)&fimu.buff[0];  CQuat qis=*(CQuat*)&fimu.buff[3];  double t=fimu.buff[7];
		if(i==0) {  // 首次星敏有效初始化
			gyroCns.Init(qis, t);
		}
		else {
			gyroCns.Update(&wm, 1, TS10);  // 陀螺姿态更新
			if(i%10==0 && norm(wm)<5.0*DPS*TS10) {  // 星敏有效且角速率较小进行星敏量测更新
				gyroCns.SetMeas(qis);
			}
			fins<<gyroCns.qib<<gyroCns.eb<<gyroCns.mu<<*(CVect3*)&gyroCns.Zk.dd[0]<<t;  // 输出
		}
		disp(i, FRQ, 100);
	}
}

void main1(void)
{
	psinsdemo(701);  return;

	CFileRdWt::Dir("D:\\ygm2020\\PSINS网站\\惯导数据\\", "D:\\psins210207\\VC60\\Data\\");
	CFileRdWt fins("ins.bin"), fkf("kf.bin");
	CFileRdSr fimu("mimuattgps.bin");  // download from: http://www.psins.org.cn/newsinfo/958984.html
	DataSensor *pDS=(DataSensor*)fimu.buff, *pDS0=&fimu.DS0;

	CKFApp kf(TS);
	kf.Init(CSINS(pDS0->att, pDS0->gpsvn, pDS0->gpspos, pDS0->t));

	for(int i=0; i<5000*FRQ; i++)
	{
		if(!fimu.load(1)) break;
		if(pDS->gpspos.i>0.1 && !hit3(pDS->t,500,600,900,1000,2000,2100))
		{
			kf.SetMeasGNSS(pDS->gpspos, pDS->gpsvn);
		}
		kf.Update(&pDS->wm, &pDS->vm, 1, TS);

		if(i%5==0||pDS->gpspos.i>0.1)
		{
			fins << kf.sins << pDS->att;
			fkf << kf;
		}

		disp(i, FRQ, 100);
	}
}
