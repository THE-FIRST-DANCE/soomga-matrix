from flask import Flask, request
from distance_compute import create_distance_matrix, solve_routing_problem, create_combined_cost_matrix

app = Flask(__name__)

@app.route('/distance', methods=['POST'])
def compute_distance():

   # JSON 데이터를 파싱합니다.
   data = request.json

   # `data['data']`로 실제 데이터에 접근합니다.
   distance_matrix = create_distance_matrix(data) 
   solution_output = solve_routing_problem(distance_matrix)

   return solution_output

if __name__ == '__main__':
   app.run('0.0.0.0',port=5000,debug=True)