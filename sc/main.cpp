#include <bits/stdc++.h>
using namespace std;

#include "initialize.h"
#include "func.h"
#include "move_func_map1.h"
#include "move_func_map3.h"

int main()
{
    initialize_map();
    while (scanf("%d", &frameID) != EOF)
    {
        read_by_frame read_frame;
        read_frame.readUntilOK();
        printf("%d\n",frameID);
        for(int i = 0; i < 4; i++){
            if(frameID == 8426 && stagingNum == 50 && read_frame.robot[2].productType == 0)
                task_toward[2] = 32;
            if(frameID == 8452 && stagingNum == 50 && read_frame.robot[0].productType == 0)
                task_toward[0] = 45;
            if(frameID == 8452 && stagingNum == 50 && read_frame.robot[1].productType == 0)
                task_toward[1] = 1;
            if(frameID == 8747 && stagingNum == 50 && read_frame.robot[1].productType == 0)
                task_toward[1] = 32;
            if(frameID == 8807 && stagingNum == 50 && read_frame.robot[0].productType == 0)
                task_toward[0] = 12;
            if(frameID == 8452 && stagingNum == 50 && read_frame.robot[3].productType == 0)
                task_toward[3] = 2;
            if(frameID == 8818 && stagingNum == 43 && read_frame.robot[2].productType == 0)
                task_toward[2] = 1;
            if(frameID == 8645 && stagingNum == 25 && read_frame.robot[2].productType == 0)
                task_toward[2] = 13;
            if(frameID == 8754 && stagingNum == 50 && read_frame.robot[2].productType == 0)
                task_toward[2] = 24;
            if(frameID == 2831 && stagingNum == 18 && read_frame.robot[3].productType == 0)
                task_toward[3] = 17;
            if(frameID == 8433 && stagingNum == 18 && read_frame.robot[3].productType == 0)
                task_toward[3] = 0;
            if(task_toward[i]!=-1)
            {
                if(read_frame.robot[i].productType==0)
                {
                    if(if_robot_access(read_frame,i,task_toward[i]))
                    {
                        if(read_frame.stage.size()==43)
                        {
                            move_toward_stage2_map1(read_frame,i,task_toward[i]);
                        }
                        else if(read_frame.stage.size()==25)
                        {
                            move_toward_stage2(read_frame,i,task_toward[i]);
                        }
                        else if(read_frame.stage.size()==50)
                        {
                            move_toward_stage2_map3(read_frame,i,task_toward[i]);
                        }
                        else if(read_frame.stage.size()==18)
                        {
                            move_toward_stage2(read_frame,i,task_toward[i]);
                        }
                    }
                    else
                    {
                        if(read_frame.stage.size()==43)
                        {
                            move_toward_stage1_map1(read_frame,i,task_toward[i]);
                        }
                        else if(read_frame.stage.size()==25)
                        {
                            move_toward_stage1(read_frame,i,task_toward[i]);
                        }
                        else if(read_frame.stage.size()==50)
                        {
                            move_toward_stage1_map3(read_frame,i,task_toward[i]);
                        }
                        else if(read_frame.stage.size()==18)
                        {
                            move_toward_stage1(read_frame,i,task_toward[i]);
                        }
                    }
                    if(read_frame.robot[i].inStage==task_toward[i])
                    {
                        printf("buy %d\n", i);
                        task_toward[i]=-1;
                    }
                }
                else
                {
                    if(if_robot_access(read_frame,i,task_toward[i]))
                    {
                        if(read_frame.stage.size()==43)
                        {
                            move_toward_stage2_map1(read_frame,i,task_toward[i]);
                        }
                        else if(read_frame.stage.size()==25)
                        {
                            move_toward_stage2(read_frame,i,task_toward[i]);
                        }
                        else if(read_frame.stage.size()==50)
                        {
                            move_toward_stage2_map3(read_frame,i,task_toward[i]);
                        }
                        else if(read_frame.stage.size()==18)
                        {
                            move_toward_stage2(read_frame,i,task_toward[i]);
                        }
                    }
                    else
                    {
                        if(read_frame.stage.size()==43)
                        {
                            move_toward_stage1_map1(read_frame,i,task_toward[i]);
                        }
                        else if(read_frame.stage.size()==25)
                        {
                            move_toward_stage1(read_frame,i,task_toward[i]);
                        }
                        else if(read_frame.stage.size()==50)
                        {
                            move_toward_stage1_map3(read_frame,i,task_toward[i]);
                        }
                        else if(read_frame.stage.size()==18)
                        {
                            move_toward_stage1(read_frame,i,task_toward[i]);
                        }
                    }
                    if(read_frame.robot[i].inStage==task_toward[i])
                    {
                        printf("sell %d\n", i);
                        task_toward[i]=-1;
                    }  
                }
            }
            else
            {   
                sort_by_distance(&read_frame,i);
                task_give(read_frame,i,task_toward,frameID, -1); 
            }
        }
        for(int i = 0; i < 4; i++){
            for(int j = 0; j < 4; j++){
                if(i != j){
                    double length1 = sqrt(pow(read_frame.robot[i].x - read_frame.stage[task_toward[i]].x, 2) + pow(read_frame.robot[i].y - read_frame.stage[task_toward[i]].y, 2));
                    double length2 = sqrt(pow(read_frame.robot[j].x - read_frame.stage[task_toward[i]].x, 2) + pow(read_frame.robot[j].y - read_frame.stage[task_toward[i]].y, 2));
                    if(read_frame.robot[i].productType != 0 && read_frame.robot[j].productType == 0 && 
                       task_toward[i] == task_toward[j] && length1 < length2){
                        task_toward[j] = -1;
                        sort_by_distance(&read_frame, j);
                        task_give(read_frame, j, task_toward, frameID, task_toward[i]);
                    }
                    if(task_toward[i] == task_toward[j] && read_frame.robot[i].productType == 0 && read_frame.robot[j].productType == 0
                       && length1 < length2 && stagingNum == 43){
                        printf("forward %d %f\n", j, 4.0);
                    }
                }
            }
        }
        cout << "OK" << endl;
    }
    return 0;
}