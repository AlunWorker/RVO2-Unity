/*
 * Simulator.cs
 * RVO2 Library C#
 *
 * Copyright 2008 University of North Carolina at Chapel Hill
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 */

using System;
using System.Collections.Generic;
using System.Threading;

namespace RVO
{
    /**
     * <summary>定义仿真器类</summary>
     */
    public class Simulator
    {
        /**
         * <summary>定义工作线程类</summary>
         */
        private class Worker
        {
            private ManualResetEvent doneEvent_;
            private int end_;
            private int start_;

            /**
             * <summary>构造并初始化工作线程。</summary>
             * <param name="start">起始索引。</param>
             * <param name="end">结束索引。</param>
             * <param name="doneEvent">完成事件。</param>
             */
            internal Worker(int start, int end, ManualResetEvent doneEvent)
            {
                start_ = start;
                end_ = end;
                doneEvent_ = doneEvent;
            }

            internal void config(int start, int end)
            {
                start_ = start;
                end_ = end;
            }

            /**
             * <summary>执行仿真步骤。</summary>
             *
             * <param name="obj">未使用。</param>
             */
            internal void step(object obj)
            {
                for (int index = start_; index < end_; ++index)
                {
                    Simulator.Instance.agents_[index].computeNeighbors();
                    Simulator.Instance.agents_[index].computeNewVelocity();
                }
                doneEvent_.Set();
            }

            /**
             * <summary>更新每个代理的二维位置和二维速度。</summary>
             *
             * <param name="obj">未使用。</param>
             */
            internal void update(object obj)
            {
                for (int index = start_; index < end_; ++index)
                {
                    Simulator.Instance.agents_[index].update();
                }

                doneEvent_.Set();
            }
        }

        internal IDictionary<int, int> agentNo2indexDict_;
        internal IDictionary<int, int> index2agentNoDict_;
        internal IList<Agent> agents_;
        internal IList<Obstacle> obstacles_;
        internal KdTree kdTree_;
        internal float timeStep_;

        private static Simulator instance_ = new Simulator();

        private Agent defaultAgent_;
        private ManualResetEvent[] doneEvents_;
        private Worker[] workers_;
        private int numWorkers_;
        private int workerAgentCount_;
        private float globalTime_;

        public static Simulator Instance
        {
            get
            {
                return instance_;
            }
        }
        
        /**
         * <summary>删除指定编号的代理。</summary>
         *
         * <param name="agentNo">代理编号。</param>
         */
        public void delAgent(int agentNo)
        {
            agents_[agentNo2indexDict_[agentNo]].needDelete_ = true;
        }

        /**
        * <summary>更新删除的代理。</summary>
        */
        void updateDeleteAgent()
        {
            bool isDelete = false;
            for (int i = agents_.Count - 1; i >= 0; i--)
            {
                if (agents_[i].needDelete_)
                {
                    agents_.RemoveAt(i);
                    isDelete = true;
                }
            }
            if (isDelete)
                onDelAgent();
        }

        static int s_totalID = 0;
        /**
         * <summary>向仿真中添加具有默认属性的新代理。</summary>
         *
         * <returns>代理的编号，未设置默认时返回-1。</returns>
         *
         * <param name="position">此代理的二维起始位置。</param>
         */
        public int addAgent(Vector2 position)
        {
            if (defaultAgent_ == null)
            {
                return -1;
            }

            Agent agent = new Agent();
            agent.id_ = s_totalID;
            s_totalID++;
            agent.maxNeighbors_ = defaultAgent_.maxNeighbors_;
            agent.maxSpeed_ = defaultAgent_.maxSpeed_;
            agent.neighborDist_ = defaultAgent_.neighborDist_;
            agent.position_ = position;
            agent.radius_ = defaultAgent_.radius_;
            agent.timeHorizon_ = defaultAgent_.timeHorizon_;
            agent.timeHorizonObst_ = defaultAgent_.timeHorizonObst_;
            agent.velocity_ = defaultAgent_.velocity_;
            agents_.Add(agent);
            onAddAgent();
            return agent.id_;
        }

        /**
         * <summary>处理删除代理时的更新操作。</summary>
         */
        void onDelAgent()
        {
            agentNo2indexDict_.Clear();
            index2agentNoDict_.Clear();

            for (int i = 0; i < agents_.Count; i++)
            {
                int agentNo = agents_[i].id_;
                agentNo2indexDict_.Add(agentNo, i);
                index2agentNoDict_.Add(i, agentNo);
            }
        }
        
        /**
         * <summary>处理添加代理时的更新操作。</summary>
         */
        void onAddAgent()
        {
            if (agents_.Count == 0)
                return;

            int index = agents_.Count - 1;
            int agentNo = agents_[index].id_;
            agentNo2indexDict_.Add(agentNo, index);
            index2agentNoDict_.Add(index, agentNo);
        }

        /**
         * <summary>向仿真中添加新的代理。</summary>
         *
         * <returns>代理的编号。</returns>
         *
         * <param name="position">此代理的二维起始位置。</param>
         * <param name="neighborDist">此代理在导航中考虑的最大距离。</param>
         * <param name="maxNeighbors">此代理在导航中考虑的最大其他代理数量。</param>
         * <param name="timeHorizon">此代理计算的安全速度的最小时间量。</param>
         * <param name="timeHorizonObst">此代理计算的安全速度相对于障碍物的最小时间量。</param>
         * <param name="radius">此代理的半径。</param>
         * <param name="maxSpeed">此代理的最大速度。</param>
         * <param name="velocity">此代理的初始二维线速度。</param>
         */
        public int addAgent(Vector2 position, float neighborDist, int maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, Vector2 velocity)
        {
            Agent agent = new Agent();
            agent.id_ = s_totalID;
            s_totalID++;
            agent.maxNeighbors_ = maxNeighbors;
            agent.maxSpeed_ = maxSpeed;
            agent.neighborDist_ = neighborDist;
            agent.position_ = position;
            agent.radius_ = radius;
            agent.timeHorizon_ = timeHorizon;
            agent.timeHorizonObst_ = timeHorizonObst;
            agent.velocity_ = velocity;
            agents_.Add(agent);
            onAddAgent();
            return agent.id_;
        }

        /**
         * <summary>向仿真中添加新的障碍物。</summary>
         *
         * <returns>障碍物的第一个顶点的编号，或当顶点数量少于两个时返回-1。</returns>
         *
         * <param name="vertices">多边形障碍物的顶点列表，按逆时针顺序排列。</param>
         *
         * <remarks>要添加"负"障碍物，例如环境的边界多边形，顶点应按顺时针顺序列出。</remarks>
         */
        public int addObstacle(IList<Vector2> vertices)
        {
            if (vertices.Count < 2)
            {
                return -1;
            }

            int obstacleNo = obstacles_.Count;

            for (int i = 0; i < vertices.Count; ++i)
            {
                Obstacle obstacle = new Obstacle();
                obstacle.point_ = vertices[i];

                if (i != 0)
                {
                    obstacle.previous_ = obstacles_[obstacles_.Count - 1];
                    obstacle.previous_.next_ = obstacle;
                }

                if (i == vertices.Count - 1)
                {
                    obstacle.next_ = obstacles_[obstacleNo];
                    obstacle.next_.previous_ = obstacle;
                }

                obstacle.direction_ = RVOMath.normalize(vertices[(i == vertices.Count - 1 ? 0 : i + 1)] - vertices[i]);

                if (vertices.Count == 2)
                {
                    obstacle.convex_ = true;
                }
                else
                {
                    obstacle.convex_ = (RVOMath.leftOf(vertices[(i == 0 ? vertices.Count - 1 : i - 1)], vertices[i], vertices[(i == vertices.Count - 1 ? 0 : i + 1)]) >= 0.0f);
                }

                obstacle.id_ = obstacles_.Count;
                obstacles_.Add(obstacle);
            }

            return obstacleNo;
        }

        /**
         * <summary>清除仿真状态。</summary>
         */
        public void Clear()
        {
            agents_ = new List<Agent>();
            agentNo2indexDict_ = new Dictionary<int, int>();
            index2agentNoDict_ = new Dictionary<int, int>();
            defaultAgent_ = null;
            kdTree_ = new KdTree();
            obstacles_ = new List<Obstacle>();
            globalTime_ = 0.0f;
            timeStep_ = 0.1f;

            SetNumWorkers(0);
        }

        /**
         * <summary>执行一次仿真步骤并更新每个代理的二维位置和二维速度。</summary>
         *
         * <returns>仿真步骤后的全球时间。</returns>
         */
        public float doStep()
        {
            updateDeleteAgent();

            if (workers_ == null)
            {
                workers_ = new Worker[numWorkers_];
                doneEvents_ = new ManualResetEvent[workers_.Length];
                workerAgentCount_ = getNumAgents();

                for (int block = 0; block < workers_.Length; ++block)
                {
                    doneEvents_[block] = new ManualResetEvent(false);
                    workers_[block] = new Worker(block * getNumAgents() / workers_.Length, (block + 1) * getNumAgents() / workers_.Length, doneEvents_[block]);
                }
            }

            if (workerAgentCount_ != getNumAgents())
            {
                workerAgentCount_ = getNumAgents();
                for (int block = 0; block < workers_.Length; ++block)
                {
                    workers_[block].config(block * getNumAgents() / workers_.Length, (block + 1) * getNumAgents() / workers_.Length);
                }
            }

            kdTree_.buildAgentTree();

            for (int block = 0; block < workers_.Length; ++block)
            {
                doneEvents_[block].Reset();
                ThreadPool.QueueUserWorkItem(workers_[block].step);
            }

            WaitHandle.WaitAll(doneEvents_);

            for (int block = 0; block < workers_.Length; ++block)
            {
                doneEvents_[block].Reset();
                ThreadPool.QueueUserWorkItem(workers_[block].update);
            }

            WaitHandle.WaitAll(doneEvents_);

            globalTime_ += timeStep_;

            return globalTime_;
        }

        /**
         * <summary>返回指定代理的指定邻居代理。</summary>
         *
         * <returns>邻居代理的编号。</returns>
         *
         * <param name="agentNo">要检索邻居的代理编号。</param>
         * <param name="neighborNo">要检索的邻居代理编号。</param>
         */
        public int getAgentAgentNeighbor(int agentNo, int neighborNo)
        {
            return agents_[agentNo2indexDict_[agentNo]].agentNeighbors_[neighborNo].Value.id_;
        }

        /**
         * <summary>返回指定代理的最大邻居数量。</summary>
         *
         * <returns>代理的当前最大邻居数量。</returns>
         *
         * <param name="agentNo">要检索最大邻居数量的代理编号。</param>
         */
        public int getAgentMaxNeighbors(int agentNo)
        {
            return agents_[agentNo2indexDict_[agentNo]].maxNeighbors_;
        }

        /**
         * <summary>返回指定代理的最大速度。</summary>
         *
         * <returns>代理的当前最大速度。</returns>
         *
         * <param name="agentNo">要检索最大速度的代理编号。</param>
         */
        public float getAgentMaxSpeed(int agentNo)
        {
            return agents_[agentNo2indexDict_[agentNo]].maxSpeed_;
        }

        /**
         * <summary>返回指定代理的最大邻居距离。</summary>
         *
         * <returns>代理的当前最大邻居距离。</returns>
         *
         * <param name="agentNo">要检索最大邻居距离的代理编号。</param>
         */
        public float getAgentNeighborDist(int agentNo)
        {
            return agents_[agentNo2indexDict_[agentNo]].neighborDist_;
        }


        /**
         * <summary>返回用于计算指定代理当前速度的邻居代理数量。</summary>
         *
         * <returns>用于计算代理当前速度的邻居代理数量。</returns>
         *
         * <param name="agentNo">要检索邻居代理数量的代理编号。</param>
         */
        public int getAgentNumAgentNeighbors(int agentNo)
        {
            return agents_[agentNo2indexDict_[agentNo]].agentNeighbors_.Count;
        }

        /**
         * <summary>返回用于计算指定代理当前速度的障碍邻居数量。</summary>
         *
         * <returns>用于计算代理当前速度的障碍邻居数量。</returns>
         *
         * <param name="agentNo">要检索障碍邻居数量的代理编号。</param>
         */
        public int getAgentNumObstacleNeighbors(int agentNo)
        {
            return agents_[agentNo2indexDict_[agentNo]].obstacleNeighbors_.Count;
        }

        /**
         * <summary>返回指定代理的指定障碍邻居。</summary>
         *
         * <returns>邻接障碍的第一个顶点编号。</returns>
         *
         * <param name="agentNo">要检索障碍邻居的代理编号。</param>
         * <param name="neighborNo">要检索的障碍邻居编号。</param>
         */
        public int getAgentObstacleNeighbor(int agentNo, int neighborNo)
        {
            return agents_[agentNo2indexDict_[agentNo]].obstacleNeighbors_[neighborNo].Value.id_;
        }

        /**
         * <summary>返回指定代理的ORCA约束。</summary>
         *
         * <returns>表示ORCA约束的线的列表。</returns>
         *
         * <param name="agentNo">要检索ORCA约束的代理编号。</param>
         *
         * <remarks>每条线的左半平面是允许速度的区域。</remarks>
         */
        public IList<Line> getAgentOrcaLines(int agentNo)
        {
            return agents_[agentNo2indexDict_[agentNo]].orcaLines_;
        }

        /**
         * <summary>返回指定代理的二维位置。</summary>
         *
         * <returns>代理的当前二维位置。</returns>
         *
         * <param name="agentNo">要检索位置的代理编号。</param>
         */
        public Vector2 getAgentPosition(int agentNo)
        {
            return agents_[agentNo2indexDict_[agentNo]].position_;
        }

        /**
         * <summary>返回指定代理的二维优选速度。</summary>
         *
         * <returns>代理的当前二维优选速度。</returns>
         *
         * <param name="agentNo">要检索优选速度的代理编号。</param>
         */
        public Vector2 getAgentPrefVelocity(int agentNo)
        {
            return agents_[agentNo2indexDict_[agentNo]].prefVelocity_;
        }

        /**
         * <summary>返回指定代理的半径。</summary>
         *
         * <returns>代理的当前半径。</returns>
         *
         * <param name="agentNo">要检索半径的代理编号。</param>
         */
        public float getAgentRadius(int agentNo)
        {
            return agents_[agentNo2indexDict_[agentNo]].radius_;
        }

        /**
         * <summary>返回指定代理的时间视野。</summary>
         *
         * <returns>代理的当前时间视野。</returns>
         *
         * <param name="agentNo">要检索时间视野的代理编号。</param>
         */
        public float getAgentTimeHorizon(int agentNo)
        {
            return agents_[agentNo2indexDict_[agentNo]].timeHorizon_;
        }


        /**
         * <summary>返回指定代理相对于障碍物的时间视野。</summary>
         *
         * <returns>代理的当前相对于障碍物的时间视野。</returns>
         *
         * <param name="agentNo">要检索时间视野的代理编号。</param>
         */
        public float getAgentTimeHorizonObst(int agentNo)
        {
            return agents_[agentNo2indexDict_[agentNo]].timeHorizonObst_;
        }

        /**
         * <summary>返回指定代理的二维线速度。</summary>
         *
         * <returns>代理的当前二维线速度。</returns>
         *
         * <param name="agentNo">要检索线速度的代理编号。</param>
         */
        public Vector2 getAgentVelocity(int agentNo)
        {
            return agents_[agentNo2indexDict_[agentNo]].velocity_;
        }

        /**
         * <summary>返回仿真的全球时间。</summary>
         *
         * <returns>仿真的当前全球时间（最初为零）。</returns>
         */
        public float getGlobalTime()
        {
            return globalTime_;
        }

        /**
         * <summary>返回仿真中的代理数量。</summary>
         *
         * <returns>仿真中的代理数量。</returns>
         */
        public int getNumAgents()
        {
            return agents_.Count;
        }

        /**
         * <summary>返回仿真中的障碍顶点数量。</summary>
         *
         * <returns>仿真中的障碍顶点数量。</returns>
         */
        public int getNumObstacleVertices()
        {
            return obstacles_.Count;
        }

        /**
         * <summary>返回工作线程数量。</summary>
         *
         * <returns>工作线程数量。</returns>
         */
        public int GetNumWorkers()
        {
            return numWorkers_;
        }

        /**
         * <summary>返回指定障碍顶点的二维位置。</summary>
         *
         * <returns>指定障碍顶点的二维位置。</returns>
         *
         * <param name="vertexNo">要检索的障碍顶点编号。</param>
         */
        public Vector2 getObstacleVertex(int vertexNo)
        {
            return obstacles_[vertexNo].point_;
        }

        /**
         * <summary>返回指定障碍顶点在其多边形中下一个障碍顶点的编号。</summary>
         *
         * <returns>指定障碍顶点在其多边形中下一个障碍顶点的编号。</returns>
         *
         * <param name="vertexNo">要检索下一个顶点的障碍顶点编号。</param>
         */
        public int getNextObstacleVertexNo(int vertexNo)
        {
            return obstacles_[vertexNo].next_.id_;
        }

        /**
         * <summary>返回指定障碍顶点在其多边形中前一个障碍顶点的编号。</summary>
         *
         * <returns>指定障碍顶点在其多边形中前一个障碍顶点的编号。</returns>
         *
         * <param name="vertexNo">要检索前一个顶点的障碍顶点编号。</param>
         */
        public int getPrevObstacleVertexNo(int vertexNo)
        {
            return obstacles_[vertexNo].previous_.id_;
        }

        /**
         * <summary>返回仿真的时间步长。</summary>
         *
         * <returns>当前仿真的时间步长。</returns>
         */
        public float getTimeStep()
        {
            return timeStep_;
        }

        /**
         * <summary>处理已添加的障碍物，使其在仿真中生效。</summary>
         *
         * <remarks>在此函数调用后添加到仿真的障碍物不会被考虑。</remarks>
         */
        public void processObstacles()
        {
            kdTree_.buildObstacleTree();
        }

        /**
         * <summary>执行两个指定点之间的可见性查询。</summary>
         *
         * <returns>布尔值，表示两点是否可见。当障碍物未处理时返回true。</returns>
         *
         * <param name="point1">查询的第一个点。</param>
         * <param name="point2">查询的第二个点。</param>
         * <param name="radius">连接两个点的线与障碍物之间的最小距离（可选）。必须为非负。</param>
         */
        public bool queryVisibility(Vector2 point1, Vector2 point2, float radius)
        {
            return kdTree_.queryVisibility(point1, point2, radius);
        }
        
        /**
         * <summary>查询与指定点接近的代理。</summary>
         *
         * <param name="point">查询的点。</param>
         * <param name="radius">接近的半径。</param>
         *
         * <returns>接近代理的编号，或-1表示无代理。</returns>
         */
        public int queryNearAgent(Vector2 point, float radius)
        {
            if (getNumAgents() == 0)
                return -1;
            return kdTree_.queryNearAgent(point, radius);
        }

        /**
         * <summary>设置添加新代理的默认属性。</summary>
         *
         * <param name="neighborDist">新代理考虑的其他代理的默认最大距离。</param>
         * <param name="maxNeighbors">新代理考虑的其他代理的默认最大数量。</param>
         * <param name="timeHorizon">新代理的速度的安全默认最小时间量。</param>
         * <param name="timeHorizonObst">相对于障碍物的安全默认最小时间量。</param>
         * <param name="radius">新代理的默认半径。</param>
         * <param name="maxSpeed">新代理的默认最大速度。</param>
         * <param name="velocity">新代理的默认初始二维线速度。</param>
         */
        public void setAgentDefaults(float neighborDist, int maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, Vector2 velocity)
        {
            if (defaultAgent_ == null)
            {
                defaultAgent_ = new Agent();
            }

            defaultAgent_.maxNeighbors_ = maxNeighbors;
            defaultAgent_.maxSpeed_ = maxSpeed;
            defaultAgent_.neighborDist_ = neighborDist;
            defaultAgent_.radius_ = radius;
            defaultAgent_.timeHorizon_ = timeHorizon;
            defaultAgent_.timeHorizonObst_ = timeHorizonObst;
            defaultAgent_.velocity_ = velocity;
        }

        /**
         * <summary>设置指定代理的最大邻居数量。</summary>
         *
         * <param name="agentNo">要修改最大邻居数量的代理编号。</param>
         * <param name="maxNeighbors">替换的最大邻居数量。</param>
         */
        public void setAgentMaxNeighbors(int agentNo, int maxNeighbors)
        {
            agents_[agentNo2indexDict_[agentNo]].maxNeighbors_ = maxNeighbors;
        }

        /**
         * <summary>设置指定代理的最大速度。</summary>
         *
         * <param name="agentNo">要修改最大速度的代理编号。</param>
         * <param name="maxSpeed">替换的最大速度。必须为非负。</param>
         */
        public void setAgentMaxSpeed(int agentNo, float maxSpeed)
        {
            agents_[agentNo2indexDict_[agentNo]].maxSpeed_ = maxSpeed;
        }
        
        /**
         * <summary>设置指定代理的最大邻居距离。</summary>
         *
         * <param name="agentNo">要修改最大邻居距离的代理编号。</param>
         * <param name="neighborDist">替换的最大邻居距离。必须为非负。</param>
         */
        public void setAgentNeighborDist(int agentNo, float neighborDist)
        {
            agents_[agentNo2indexDict_[agentNo]].neighborDist_ = neighborDist;
        }

        /**
         * <summary>设置指定代理的二维位置。</summary>
         *
         * <param name="agentNo">要修改位置的代理编号。</param>
         * <param name="position">替换的二维位置。</param>
         */
        public void setAgentPosition(int agentNo, Vector2 position)
        {
            agents_[agentNo2indexDict_[agentNo]].position_ = position;
        }

        /**
         * <summary>设置指定代理的二维优选速度。</summary>
         *
         * <param name="agentNo">要修改优选速度的代理编号。</param>
         * <param name="prefVelocity">替换的二维优选速度。</param>
         */
        public void setAgentPrefVelocity(int agentNo, Vector2 prefVelocity)
        {
            agents_[agentNo2indexDict_[agentNo]].prefVelocity_ = prefVelocity;
        }

        /**
         * <summary>设置指定代理的半径。</summary>
         *
         * <param name="agentNo">要修改半径的代理编号。</param>
         * <param name="radius">替换的半径。必须为非负。</param>
         */
        public void setAgentRadius(int agentNo, float radius)
        {
            agents_[agentNo2indexDict_[agentNo]].radius_ = radius;
        }


        /**
         * <summary>设置指定代理的时间视野。</summary>
         *
         * <param name="agentNo">要修改时间视野的代理编号。</param>
         * <param name="timeHorizon">替换的时间视野。必须为正。</param>
         */
        public void setAgentTimeHorizon(int agentNo, float timeHorizon)
        {
            agents_[agentNo2indexDict_[agentNo]].timeHorizon_ = timeHorizon;
        }

        /**
         * <summary>设置指定代理相对于障碍物的时间视野。</summary>
         *
         * <param name="agentNo">要修改时间视野的代理编号。</param>
         * <param name="timeHorizonObst">替换的时间视野相对于障碍物。必须为正。</param>
         */
        public void setAgentTimeHorizonObst(int agentNo, float timeHorizonObst)
        {
            agents_[agentNo2indexDict_[agentNo]].timeHorizonObst_ = timeHorizonObst;
        }
        
        /**
         * <summary>设置指定代理的二维线速度。</summary>
         *
         * <param name="agentNo">要修改线速度的代理编号。</param>
         * <param name="velocity">替换的二维线速度。</param>
         */
        public void setAgentVelocity(int agentNo, Vector2 velocity)
        {
            agents_[agentNo2indexDict_[agentNo]].velocity_ = velocity;
        }
        
        /**
         * <summary>设置仿真的全球时间。</summary>
         *
         * <param name="globalTime">仿真的全球时间。</param>
         */
        public void setGlobalTime(float globalTime)
        {
            globalTime_ = globalTime;
        }

        /**
         * <summary>设置工作线程的数量。</summary>
         *
         * <param name="numWorkers">工作线程的数量。</param>
         */
        public void SetNumWorkers(int numWorkers)
        {
            numWorkers_ = numWorkers;

            if (numWorkers_ <= 0)
            {
                int completionPorts;
                ThreadPool.GetMinThreads(out numWorkers_, out completionPorts);
            }
            workers_ = null;
            workerAgentCount_ = 0;
        }

        /**
         * <summary>设置仿真的时间步长。</summary>
         *
         * <param name="timeStep">仿真的时间步长。必须为正。</param>
         */
        public void setTimeStep(float timeStep)
        {
            timeStep_ = timeStep;
        }

        /**
         * <summary>构造并初始化一个仿真。</summary>
         */
        private Simulator()
        {
            Clear();
        }
    }
}
