from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

# 거리 매트릭스 생성
def create_distance_matrix(data):
  # 최대'index 값을 기반으로 매트릭스 크기 결정
  num_places = max(item['index'] for item in data) + 1
  # 거리 매트릭스 초기화
  distance_matrix = [[0 for _ in range(num_places)] for _ in range(num_places)]
  
  # 거리 정보를 매트릭스에 채우기
  for item in data:
    origin_id = item['index']
    destination_id = [d['index'] for d in data if d['origin'] == item['destination']][0]  # 목적지'index 찾기
    distance_matrix[origin_id][destination_id] = item['distance']

  return distance_matrix

# 조합 비용 매트릭스 생성
def create_combined_cost_matrix(data, distance_weight, time_weight):
  num_places = max(item['index'] for item in data) + 1
  combined_cost_matrix = [[0 for _ in range(num_places)] for _ in range(num_places)]
  
  for item in data:
    origin_id = item['index']
    destination_ids = [d['index'] for d in data if d['origin'] == item['destination']]
    if destination_ids:
      destination_id = destination_ids[0]
      distance = float(item['distance'])
      time = float(item['duration'])
      combined_cost = (distance_weight * distance) + (time_weight * time) 
      combined_cost_matrix[origin_id][destination_id] = combined_cost

  print(combined_cost_matrix)
  
  return combined_cost_matrix

# 최적화 문제 해결
def solve_routing_problem(distance_matrix):
  # 데이터 모델 생성
  data = {}
  data['distance_matrix'] = distance_matrix
  data['num_vehicles'] = 1
  data['depot'] = 0

  # 라우팅 인덱스 매니저 및 모델 생성
  manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), data['num_vehicles'], data['depot'])
  routing = pywrapcp.RoutingModel(manager)

  # 거리 콜백
  def distance_callback(from_index, to_index):
    return data['distance_matrix'][manager.IndexToNode(from_index)][manager.IndexToNode(to_index)]

  transit_callback_index = routing.RegisterTransitCallback(distance_callback)
  routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

  # 최적화 실행
  search_parameters = pywrapcp.DefaultRoutingSearchParameters()
  search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
  search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
  search_parameters.time_limit.seconds = 2

  solution = routing.SolveWithParameters(search_parameters)

  # 결과 출력
  index = routing.Start(0)
  plan_output = []
  while not routing.IsEnd(index):
    plan_output.append(manager.IndexToNode(index))
    index = solution.Value(routing.NextVar(index))

  plan_output.append(manager.IndexToNode(index))

  return plan_output
