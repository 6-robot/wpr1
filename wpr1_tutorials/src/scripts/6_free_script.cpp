#include "6_free_script.h"
CFreeScript::CFreeScript()
{
    
}

CFreeScript::~CFreeScript()
{

}

void CFreeScript::Queue()
{
    stAct newAct;

    newAct.nAct = ACT_SPEAK;
    newAct.strTarget = "非限定项目开始";
    newAct.nDuration = 3;
    arAct.push_back(newAct);

    newAct.nAct = ACT_LISTEN;
    newAct.strTarget = "机器人";
    arAct.push_back(newAct);
    
    newAct.nAct = ACT_SPEAK;
    newAct.strTarget = "你好,请问你需要什么?";
    newAct.nDuration = 5;
    arAct.push_back(newAct);

    newAct.nAct = ACT_LISTEN;
    newAct.strTarget = "可乐";
    newAct.nDuration = 3;
    arAct.push_back(newAct);

    newAct.nAct = ACT_SPEAK;
    newAct.strTarget = "请问你是需要一瓶可乐吗?";
    newAct.nDuration = 5;
    arAct.push_back(newAct);

    newAct.nAct = ACT_LISTEN;
    newAct.strTarget = "是的";
    newAct.nDuration = 3;
    arAct.push_back(newAct);

    newAct.nAct = ACT_SPEAK;
    newAct.strTarget = "好的,我这就去为你取来";
    newAct.nDuration = 3;
    arAct.push_back(newAct);

    newAct.nAct = ACT_GOTO;
    newAct.strTarget = "kitchen";
    arAct.push_back(newAct);
}