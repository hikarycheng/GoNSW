#ifndef GONSW_
#define GONSW_

#define _CRT_SECURE_NO_WARNINGS
// The total runtime for this code is O((mk)^2), m for m schedules in this network, and k for k stops in each line,
//detailed calculation has been included within the functions.

#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "WGraph.h"
#include "PQueue.h"

#define MAX_STOP_NAME_LEN 32
#define INFINITE 999999

typedef struct StopNodeT* Stop;

typedef struct StopNodeT
{
	int scheduleSeq;
	int seq;
	char stopName[32];
	char arriveTime[5];
}StopNodeT;

bool isSameStr(char str1[], char str2[])
{
	return strcmp(str1, str2) == 0;
}

Stop newStopNode(char stopName[], char arriveTime[], int seq, int scheduleSeq)
{
	Stop node = malloc(sizeof(StopNodeT));
	assert(node != NULL);
	strcpy(node->arriveTime, arriveTime);
	node->scheduleSeq = scheduleSeq;
	node->seq = seq;
	strcpy(node->stopName, stopName);
	return node;
}

bool isSameStop(Stop stop1, Stop stop2)
{
	return (strcmp(stop1->stopName, stop2->stopName) == 0);
}

bool isEarlierDepart(Stop stop1, Stop stop2)
{
	return atoi(stop1->arriveTime) < atoi(stop2->arriveTime);
}

bool isNotLaterDepart(Stop stop1, Stop stop2)
{
	return atoi(stop1->arriveTime) <= atoi(stop2->arriveTime);
}

bool isLaterDepart(Stop stop1, Stop stop2)
{
	return atoi(stop1->arriveTime) > atoi(stop2->arriveTime);
}

bool inSameSchedule(Stop stop1, Stop stop2)
{
	return stop1->scheduleSeq == stop2->scheduleSeq;
}

bool departBefore(Stop stop, char departTime[])
{
	return atoi(stop->arriveTime) < atoi(departTime);
}

bool departAt(Stop stop, char departTime[])
{
	return atoi(stop->arriveTime) == atoi(departTime);
}

bool isStop(Stop s, char stopName[])
{
	return (strcmp(s->stopName, stopName) == 0);
}

int getWeight(Stop stop1, Stop stop2)
{
	return atoi(stop2->arriveTime) - atoi(stop1->arriveTime);
}

int getDepartTime(Stop s)
{
	return atoi(s->arriveTime);
}

Graph networkInit(Stop* stopsInfo, int numOfStops)
{
	Graph map = newGraph(numOfStops);
	Edge e;

	// the variable numOfStops is the sum of total bus stop in each schedule, assume there are 
	// m schedules inputed, and k stops for each schedule, then the outer loop runs O(mk) times,
	// inner loop runs O(nm) times, the total run time is O((mk)^2)
	for (int i = 0; i < numOfStops; i++)
	{
		// add edge from same stop in different schedules
		for (int j = 0; j < numOfStops; j++)
		{
			if (i == j)
				continue;
			if (isSameStop(stopsInfo[i], stopsInfo[j]) && isNotLaterDepart(stopsInfo[i], stopsInfo[j]))
			{
				e.v = i;
				e.w = j;
				e.weight = getDepartTime(stopsInfo[j]);
				//e.weight = getWeight(stopsInfo[i], stopsInfo[j]);
				insertEdge(map, e);
			}
		}

		if (i < numOfStops - 1 && inSameSchedule(stopsInfo[i], stopsInfo[i + 1]))
		{
			e.v = i;
			e.w = i + 1;
			//e.weight = getWeight(stopsInfo[i], stopsInfo[i + 1]);
			e.weight = getDepartTime(stopsInfo[i + 1]);
			insertEdge(map, e);
		}
	}
	//showGraph(map);
	return map;
}

Vertex getSource(int pred[], Vertex v)
{
	Vertex curr = v;
	Vertex prev = curr;
	while (prev > -1)
	{
		curr = prev;
		prev = pred[prev];
	}
	return curr;
}

bool sourceDepartEarlierThan(Stop* stopsInfo, int pred[], Vertex stop1, Vertex stop2)
{
	Vertex source1 = getSource(pred, stop1);
	Vertex source2 = getSource(pred, stop2);
	return isEarlierDepart(stopsInfo[source1], stopsInfo[source2]);
}

bool sourceDepartLaterThan(Stop* stopsInfo, int pred[], Vertex stop1, Vertex stop2)
{
	Vertex source1 = getSource(pred, stop1);
	Vertex source2 = getSource(pred, stop2);
	return isLaterDepart(stopsInfo[source1], stopsInfo[source2]);
}

// run time is O(mk), m for m schedules, k for total stop in a single line
void printSolution(Stop* stopsInfo, int  pred[], int destination, int nV)
{
	int prev = pred[destination];
	int curr = destination;
	int path[MAX_NODES];
	bool scheduleSeqFlag[MAX_NODES];
	int count = -1;

	//mk points, run time is O(mk)
	for (int i = 0; i < nV; i++)
	{
		path[i] = -1;
		scheduleSeqFlag[i] = true;
	}

	//there could be mk previous stops, then run time is O(mk)
	while (prev != -1)
	{
		int stopSSeq = stopsInfo[curr]->scheduleSeq;
		int prevSSeq = stopsInfo[prev]->scheduleSeq;

		path[++count] = prev;
		if (stopSSeq != prevSSeq)
			scheduleSeqFlag[count] = false;

		curr = prev;
		prev = pred[prev];
	}
	if (count >= 0)
	{
		int stop;

		//there are mk stops at most, then runtime is O(mk)
		while (count >= 0)
		{
			stop = path[count];
			printf("%s %s\n", stopsInfo[stop]->arriveTime, stopsInfo[stop]->stopName);

			if (!scheduleSeqFlag[count])
				printf("Change at %s\n", stopsInfo[stop]->stopName);

			count--;
		}
	}
	printf("%s %s\n", stopsInfo[destination]->arriveTime, stopsInfo[destination]->stopName);
	return;
}

// runtime for this func is (mk)^2+(mk)^2, O((mk)^2)
void findRoute(Graph networkMap, Stop* stopsInfo, char* from, char* to, char* at)
{
	int  dist[MAX_NODES];
	int  pred[MAX_NODES];
	bool vSet[MAX_NODES];  // vSet[v] = true <=> v has not been processed
	bool destination[MAX_NODES];
	bool source[MAX_NODES];
	PQueueInit();

	int numOfStops = numOfVertices(networkMap);
	int nSources = 0, nPossibleDestination = 0;

	// mk nodes then run time of outer loop is O(mk), and O(n) for enqueue the pqqueue with n nodes
	// there are 0 node in the queue at the beginning, then the run time is 1+...+mk = (1+mk)mk/2, 
	// total runtime is O((mk)^2)
	for (int i = 0; i < numOfStops; i++)
	{
		destination[i] = false;
		source[i] = false;
		dist[i] = INFINITE;
		pred[i] = -1;
		vSet[i] = true;
		joinPQueue(i);
		// PQ cost O(mk) for enqueuing, mk for mk nodes that already in pqueue.
		if (isStop(stopsInfo[i], from) && !departBefore(stopsInfo[i], at))
		{
			source[i] = true;
			nSources++;
		}
		if (isStop(stopsInfo[i], to) && !departBefore(stopsInfo[i], at))
		{
			destination[i] = true;
			nPossibleDestination++;
		}
	}

	if (nSources == 0 || nPossibleDestination == 0)
	{
		printf("No connection found.\n");
		return;
	}

	// mk nodes then run time of loop is O(mk)
	for (int i = 0; i < numOfStops; i++)
	{
		if (source[i])
			dist[i] = atoi(stopsInfo[i]->arriveTime);
	}

	// mk nodes then run time of outer loop is O(mk)
	// the for loop cost O((mk)^2) in total, mk*(mk + mk + mk)
	for (int count = 0; count < numOfStops; count++)
	{
		if (PQueueIsEmpty())
			break;
		// O(n) for n nodes in queue
		int bestVertex = leavePQueue(dist);

		if (dist[bestVertex] == INFINITE)
		{
			printf("No connection found.\n");
			return;
		}

		if (destination[bestVertex]) //isStop(stopsInfo[bestVertex], to)
		{
			Stop finDes = stopsInfo[bestVertex];
			// Stop departSource = stopsInfo[getSource(pred, bestVertex)];

			if (!PQueueIsEmpty())
			{
				int otherProssible = leavePQueue(dist);
				// could be up to O(mk) times for the while loop
				while (departAt(stopsInfo[otherProssible], finDes->arriveTime))
				{
					if (!destination[bestVertex])  //isStop(stopsInfo[otherProssible], to)
					{
						if (PQueueIsEmpty())
							break;
						otherProssible = leavePQueue(dist);
						continue;
					}
					if (sourceDepartLaterThan(stopsInfo, pred, otherProssible, bestVertex) && destination[otherProssible])
					{
						bestVertex = otherProssible;
						finDes = stopsInfo[bestVertex];
						// departSource = stopsInfo[getSource(pred, bestVertex)];
					}
					if (PQueueIsEmpty())
						break;
					otherProssible = leavePQueue(dist);
				}
			}

			// runtime of print func is O(mk), calculated within the func
			printSolution(stopsInfo, pred, bestVertex, numOfStops);
			return;
		}

		vSet[bestVertex] = false;
		// O(mk) for this loop, mk is the number of total stops
		for (int i = 0; i < numOfStops; i++)
		{
			int weight = adjacent(networkMap, bestVertex, i);

			if (weight < dist[i] && weight != 0 && source[i] == false)
			{
				dist[i] = weight ;
				pred[i] = bestVertex;
			}
			else if (weight != 0 && sourceDepartLaterThan(stopsInfo, pred, bestVertex, i) == true && source[i] == false)
			{
				dist[i] = weight ;
				pred[i] = bestVertex;
			}
		}
	}
}

// runtime for main is O((mk)^2), detail presented within function
int main()
{
	int totalStopsInCity, numSchedules;
	int numOfStops = 0;
	Stop* stopsInfo = malloc(MAX_NODES * sizeof(Stop));

	printf("Enter the total number of stops on the network: ");
	scanf("%d", &totalStopsInCity);

	// there are n stops in city, scanning it costs O(n)
	for (int i = 0; i < totalStopsInCity; i++)
	{
		char stopName[MAX_STOP_NAME_LEN];
		scanf("%s", stopName);
	}

	printf("Enter the number of schedules: ");
	scanf("%d", &numSchedules);

	// there are m schedules, for loop runs O(n) times
	for (int i = 0; i < numSchedules; i++)
	{
		int nStopsInThisSchedule;

		printf("Enter the number of stops: ");
		scanf("%d", &nStopsInThisSchedule);

		// In this for loop, runs O(k) times for each schedule
		for (int j = 0; j < nStopsInThisSchedule; j++)
		{
			char time[5];
			char stopName[MAX_STOP_NAME_LEN];

			scanf("%s", time);
			scanf("%s", stopName);

			Stop newStop = newStopNode(stopName, time, numOfStops, i);
			stopsInfo[numOfStops] = newStop;

			numOfStops++;
		}
	} // Therefore, the the run time of this big loop is O(nk)

	 //Show stopsInfo
	 for (int i = 0; i < numOfStops; i++)
	 {
	 	printf("stopName: %s, time: %s, seq: %d, scheduleSeq: %d\n", stopsInfo[i]->stopName, stopsInfo[i]->arriveTime, stopsInfo[i]->seq, stopsInfo[i]->scheduleSeq);
	 }

	Graph networkMap = networkInit(stopsInfo, numOfStops);

	char from[MAX_STOP_NAME_LEN];
	putchar('\n');
	printf("From: ");
	scanf("%s", from);
	while (strcmp(from, "done") != 0)
	{
		char to[MAX_STOP_NAME_LEN], departAt[5];
		printf("To: ");
		scanf("%s", to);
		printf("Depart at: ");
		scanf("%s", departAt);
		putchar('\n');

		// runtime is O((mk)^2), mk for total nodes in network, detailed calculation is presented within function
		findRoute(networkMap, stopsInfo, from, to, departAt);
		putchar('\n');

		printf("From: ");
		scanf("%s", from);
	}
	printf("Thank you for using goNSW.\n");

	// O((mk)^2) for mk stops in total
	for (int i = 0; i < numOfStops; i++)
	{
		free(stopsInfo[i]);  //free newStop
	}

	free(stopsInfo);
	freeGraph(networkMap);
	// TODO free stopsInfo

	return 0;
}

#endif // !GONSW_
