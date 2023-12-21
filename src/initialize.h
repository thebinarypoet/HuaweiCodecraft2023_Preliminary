#define robot_radius_empty 0.45
#define robot_radius_full  0.53
#define pi acos(-1)

int frameID;
int money=200000;
int stagingNum=0;
int task_toward[4]={-1,-1,-1,-1};//robot的任务状态值

void initialize_map()//初始化地图数据
{
    char line[100];
    while(fgets(line,100,stdin))
    {
        if(line[0]=='O'&&line[1]=='K')
        {
            cout<<"OK"<<endl;
            return;
        }
    }
}


struct staging//定义工作台
{
    int type = 0, productStatus = 0, materialStatus = 0, remainTime = 0, value = 0;
    double x = 0, y = 0;
    bool operator < (const staging &stage)const {
        if(value != stage.value) return value > stage.value;
        return 0;
    }
};

struct Robot//定义机器人
{
    int inStage = 0, productType = 0;
    double timeValue = 0, bangValue = 0, angelSpeed = 0, forward = 0, x = 0, y = 0, xLineSpeed = 0, yLineSpeed = 0;
};


class read_by_frame//定义处理每一帧数据的函数
{
public:
    void readUntilOK()
    {
        cin >> money;
        cin >> stagingNum;
        robot.resize(4);
        stage.resize(stagingNum);
        stage_type.resize(10);
        for (int i = 0; i < stagingNum; i++)
        {
            cin >> stage[i].type >> stage[i].x >> stage[i].y >> stage[i].remainTime;
            cin >> stage[i].materialStatus >> stage[i].productStatus;
            if(stage[i].type == 1 || stage[i].type == 2 || stage[i].type == 3)
                stage[i].value = 0;
            if(stage[i].type == 4 || stage[i].type == 5 || stage[i].type == 6)
                stage[i].value = 500;
            if(stage[i].type == 7 && stagingNum == 18)
                stage[i].value = 1000; //初始权值赋值，占比为500
            else if(stage[i].type == 7)
                stage[i].value = 1000;
            stage_value_order.push_back(i);
            stage_type[stage[i].type].push_back(i);
        }
        for (int i = 0; i < 4; i++)
        {
            cin >> robot[i].inStage >> robot[i].productType >> robot[i].timeValue;
            cin >> robot[i].bangValue >> robot[i].angelSpeed >> robot[i].xLineSpeed;
            cin >> robot[i].yLineSpeed >> robot[i].forward >> robot[i].x >> robot[i].y;
        }
        string s;
        cin>>s;
    }
    vector<Robot> robot;//存放机器人本体
    vector<staging> stage;//存放工作台本体
    vector<int> stage_value_order;//存放按工作台权值排名的工作台ID
    vector<vector<int>> stage_type;//存放按工作台类型分类的工作台ID
};