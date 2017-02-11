-module(astar).
-compile(export_all).
-include("records.hrl").

-define(PI,math:pi()).
-define(f(X), is_float(X)).	
-record(sector,{
    coord,
    description=[],
    r
}).
-record(state,{
    agent_position,
    agent_direction=90,
    sectors=[],
    tot_runs=100,
    run_index=0,
    switch_event,
    switched=false,
    step_index=0,
    fitness_acc=50
}).

sim(ExoSelf_PId)->
	io:format("Starting dtm_sim~n"),
	random:seed(now()),
	%io:format("Starting pb_sim:~p~n",[self()]),
	sim(ExoSelf_PId,#dtm_state{switch_event=35+random:uniform(30), sectors=set_tmaze_sectors()}).

sim(ExoSelf_PId,S) when (S#dtm_state.run_index == S#dtm_state.switch_event) and (S#dtm_state.switched==false)->
	%io:format("Switch event:~p~n",[S#dtm_state.switch_event]),
	Sectors=S#dtm_state.sectors,
	SectorA=lists:keyfind([1,1],2,Sectors),
	SectorB=lists:keyfind([-1,1],2,Sectors),
	U_SectorA=SectorA#dtm_sector{r=SectorB#dtm_sector.r},
	U_SectorB=SectorB#dtm_sector{r=SectorA#dtm_sector.r},
	U_Sectors=lists:keyreplace([-1,1],2,lists:keyreplace([1,1],2,Sectors,U_SectorA),U_SectorB),
	?MODULE:sim(ExoSelf_PId,S#dtm_state{sectors=U_Sectors, switched=true});
sim(ExoSelf_PId,S)->
	receive
		{From_PId,sense,Parameters}->
			%io:format("Sense:~p~n",[Parameters]),
			APos = S#dtm_state.agent_position,
			ADir = S#dtm_state.agent_direction,
			Sector=lists:keyfind(APos,2,S#dtm_state.sectors),
			{ADir,NextSec,RangeSense} = lists:keyfind(ADir,1,Sector#dtm_sector.description),
			SenseSignal=case Parameters of
				[all] ->
					RangeSense++[Sector#dtm_sector.r];
				[range_sense]->
					RangeSense;
				[reward] ->
					[Sector#dtm_sector.r]
			end,
			%io:format("Position:~p SenseSignal:~p ",[APos,SenseSignal]),
			From_PId ! {self(),percept,SenseSignal},
			?MODULE:dtm_sim(ExoSelf_PId,S);
		{From_PId,move,_Parameters,[Move]}->
			%timer:sleep(500),
			APos = S#dtm_state.agent_position,
			ADir = S#dtm_state.agent_direction,
			Sector=lists:keyfind(APos,2,S#dtm_state.sectors),
			U_StepIndex = S#dtm_state.step_index+1,
			%io:format("S:~p~n",[S]),
			%io:format("Move:~p StepIndex:~p RunIndex:~p~n",[Move,U_StepIndex,S#dtm_state.run_index]),
			{ADir,NextSec,RangeSense} = lists:keyfind(ADir,1,Sector#dtm_sector.description),
			RewardSector1 = lists:keyfind([1,1],2,S#dtm_state.sectors),
			RewardSector2 = lists:keyfind([-1,1],2,S#dtm_state.sectors),
			if
				(APos == [1,1]) or (APos == [-1,1]) ->
					Updated_RunIndex=S#dtm_state.run_index+1,
					case Updated_RunIndex >= S#dtm_state.tot_runs of
						true ->
							From_PId ! {self(),S#dtm_state.fitness_acc+Sector#dtm_sector.r,1},
							%io:format("Ok1:~p~n",[S#dtm_state.fitness_acc]),
							U_S = #dtm_state{
								switch_event=35+random:uniform(30),
								sectors=set_tmaze_sectors(),
								switched=false,
								agent_position=[0,0],
								agent_direction=90,
								run_index=0,
								step_index = 0,
								fitness_acc=50
							},
							?MODULE:sim(ExoSelf_PId,U_S);
						false ->
							From_PId ! {self(),0,0},
							U_S = S#dtm_state{
								agent_position=[0,0],
								agent_direction=90,
								run_index=Updated_RunIndex,
								step_index = 0,
								fitness_acc = S#dtm_state.fitness_acc+Sector#dtm_sector.r
							},
							?MODULE:sim(ExoSelf_PId,U_S)
					end;
				Move > 0.33 -> %clockwise
					NewDir=(S#dtm_state.agent_direction + 270) rem 360,
					{NewDir,NewNextSec,NewRangeSense} = lists:keyfind(NewDir,1,Sector#dtm_sector.description),
					From_PId ! {self(),0,0},
					U_S = move(ExoSelf_PId,From_PId,S#dtm_state{
						agent_direction=NewDir
					},NewNextSec,U_StepIndex),
					?MODULE:sim(ExoSelf_PId,U_S);
				Move < -0.33 -> %counterclockwise
					NewDir=(S#dtm_state.agent_direction + 90) rem 360,
					{NewDir,NewNextSec,NewRangeSense} = lists:keyfind(NewDir,1,Sector#dtm_sector.description),
					From_PId ! {self(),0,0},
					U_S = move(ExoSelf_PId,From_PId,S#dtm_state{
						agent_direction=NewDir
					},NewNextSec,U_StepIndex),
					?MODULE:sim(ExoSelf_PId,U_S);
				true -> %forward
					move(ExoSelf_PId,From_PId,S,NextSec,U_StepIndex)
			end;
		{ExoSelf_PId,terminate} ->
			ok
	end.

	move(ExoSelf_PId,From_PId,S,NextSec,U_StepIndex)->
		case NextSec of
			[] -> %wall crash/restart_state
				Updated_RunIndex = S#dtm_state.run_index+1,
				case Updated_RunIndex >= S#dtm_state.tot_runs of
					true ->
						From_PId ! {self(),S#dtm_state.fitness_acc-0.4,1},
						%io:format("Ok:~p~n",[S#dtm_state.fitness_acc-0.4]),
						U_S = #dtm_state{
							switch_event=35+random:uniform(30),
							sectors=set_tmaze_sectors(),
							switched=false,
							run_index=0,
							step_index=0,
							agent_position=[0,0],
							agent_direction=90,
							fitness_acc=50
						},
						?MODULE:sim(ExoSelf_PId,U_S);
					false ->
						From_PId ! {self(),0,0},
						U_S = S#dtm_state{
							agent_position=[0,0],
							agent_direction=90,
							run_index=Updated_RunIndex,
							step_index = 0,
							fitness_acc = S#dtm_state.fitness_acc-0.4
						},
						?MODULE:sim(ExoSelf_PId,U_S)
					end;
			_ -> %move
				From_PId ! {self(),0,0},
				U_S = S#dtm_state{
					agent_position=NextSec,
					step_index = U_StepIndex
				},
				?MODULE:sim(ExoSelf_PId,U_S)
		end.

set_maze_sectors()->
	Sectors = [
	#dtm_sector{id=[0,0],description=[{0,[],[1,0,0]},{90,[0,1],[0,1,0]},{180,[],[0,0,1]},{270,[],[0,0,0]}],r=0},
	#dtm_sector{id=[0,1],description=[{0,[1,1],[0,1,1]},{90,[],[1,0,1]},{180,[-1,1],[1,1,0]},{270,[0,0],[1,1,1]}],r=0},
	#dtm_sector{id=[1,1],description=[{0,[],[0,0,0]},{90,[],[2,0,0]},{180,[0,1],[0,2,0]},{270,[],[0,0,2]}],r=1},
	#dtm_sector{id=[-1,1],description=[{0,[0,1],[0,2,0]},{90,[],[0,0,2]},{180,[],[0,0,0]},{270,[],[2,0,0]}],r=0.2}
	].

distance(Vector1,Vector2)->
	distance(Vector1,Vector2,0).	
distance([Val1|Vector1],[Val2|Vector2],Acc)->
	distance(Vector1,Vector2,Acc+math:pow(Val2-Val1,2));
distance([],[],Acc)->
	math:sqrt(Acc).
