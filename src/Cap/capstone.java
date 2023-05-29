package Cap;
import ilog.concert.*;
import ilog.cplex.*;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
public class capstone {
	public static void main(String[] args) throws IOException {
		// TODO Auto-generated method stub
		
		model1();
	}
	
	public static void model1() throws IOException{
		GetData.main(null);
		int n = GetData.n; //노드 수
		int K = GetData.K; //차량의 수
		int Q = GetData.Q; //차량 용량
		
		double[][] c = new double [n][n]; //비용
		int[] p = new int[n]; //pickup 개수
		int[] d = new int[n]; //delivery 개수
		double[][][] std = new double[n][n][K]; //표준편차
		double[][][] v = new double[n][n][K]; //노드간 차량의 속력
		double[][] D = new double[n][n]; //각 노드간 거리
		double s[] = new double[n]; //pickup, delivery 시간
		
		D = GetData.d;
		d = GetData.delivery;
		p = GetData.pickup;
		
		for(int k=0; k<K; k++) {
			for(int i=0; i<n; i++) {
				for(int j=0; j<n; j++) {
					v[i][j][k] = GetData.speed[i][j];
					std[i][j][k] = GetData.std[i][j];
				}
			}
		}
		
		for(int i=0; i<n;i++) {
			for(int j=0; j<n; j++) {
				if(i==j)
					c[i][j] = 0;
				else c[i][j] = (D[i][j]*10)/(GetData.speed[i][j]/60.*1000);
			}
		}
		
//		for(int i=0; i<n;i++) {
//			for(int j=0; j<n; j++) {
//				c[i][j] = D[i][j];
//			}
//		}
		
		double PI = 1.2816;
		double a,b,e;
		a = 1.5;
		b = 1/3;
		e = 0.5;
		
		for(int i=0; i<n; i++) {
			s[i] = (a+e)*d[i] + b*p[i];
		}
		
		double[] B = new double[K];
		for(int k=0; k<K; k++) {
			B[k] = 360;
		}
		
		
		
		String fileName = "Result K="+K+" PI="+PI+" B[k]="+(int)B[0]+" Q="+(int)Q;
		String filePath = "/Users/hijieung/Desktop/Opt_Lab/Result/";
		String fileType = ".txt";
		try {
			IloCplex cplex = new IloCplex();
			
			//결정변수
			IloNumVar[][][] x = new IloNumVar[n][n][K];
			IloNumVar[][] y = new IloNumVar[n][];
			IloNumVar[][] z = new IloNumVar[n][];
			IloNumVar[] w = new IloNumVar[K];
			
			//비음 제약식
			for(int i = 0; i<n; i++) {
				for(int j = 0; j<n; j++) {
					x[i][j] = cplex.numVarArray(K, 0, 1, IloNumVarType.Int);
				}
			}
			for(int i=0; i<n; i++) {
				y[i] = cplex.numVarArray(n, 0, Double.MAX_VALUE);
				z[i] = cplex.numVarArray(n, 0, Double.MAX_VALUE);
			}
			for(int i=0; i<K; i++) {
				w[i] = cplex.numVar(0, Double.MAX_VALUE);
			}
			
			
			//목적함수
			IloLinearNumExpr objective = cplex.linearNumExpr();
			for(int i=0; i<n; i++) {
				for(int j=0; j<n; j++) {
					for(int k=0; k<K; k++) {
						objective.addTerm(s[j], x[i][j][k]);
						objective.addTerm((c[i][j]), x[i][j][k]);
						
					}
				}
			}
			
			cplex.addMinimize(objective);
			
			
			//제약식
			
			//제약식(1)
			IloLinearNumExpr[] con1 = new IloLinearNumExpr[n];
			for(int i=1; i<n; i++) {
				con1[i] = cplex.linearNumExpr();
				for(int j=0; j<n; j++) {
					for(int k=0; k<K; k++) {
						con1[i].addTerm(1.0, x[i][j][k]);
					}
				}
				cplex.addEq(con1[i], 1);
				
			}
			
			//제약식(2)
			IloLinearNumExpr[][] con2 = new IloLinearNumExpr[n][K];
			for(int i=0; i<n; i++) {
				for(int k=0; k<K; k++) {
					con2[i][k] = cplex.linearNumExpr();
					for(int j=0; j<n; j++) {
						con2[i][k].addTerm(1.0, x[i][j][k]);
						con2[i][k].addTerm(-1, x[j][i][k]);
					}
					cplex.addEq(con2[i][k],0);
				}
			}

			
			//제약식(3)
			IloLinearNumExpr[] con3 = new IloLinearNumExpr[K];
			for(int k=0; k<K; k++) {
				con3[k] = cplex.linearNumExpr();
				for(int i=0; i<n; i++) {
					con3[k].addTerm(1.0, x[0][i][k]);
				}
				cplex.addLe(con3[k], 1);
			}
			
			//제약식(4)
			IloLinearNumExpr[][] con4 = new IloLinearNumExpr[n][n];
			for(int i=0; i<n; i++) {
				for(int j=0; j<n; j++) {
					if(i==j) continue;
					con4[i][j] = cplex.linearNumExpr();
					for(int k=0; k<K; k++) {
						con4[i][j].addTerm(Q, x[i][j][k]);
					}
					cplex.addLe(cplex.sum(y[i][j],z[i][j]), con4[i][j]);
				}
			}
			
			//제약식(5)
			IloLinearNumExpr[] con5 = new IloLinearNumExpr[n];
			for(int i=1; i<n; i++) {
				con5[i] = cplex.linearNumExpr();
				for(int j=0; j<n; j++) {
					con5[i].addTerm(y[i][j], 1);
					con5[i].addTerm(y[j][i], -1);
				}
				cplex.addEq(con5[i], p[i]);
			}
			
			//제약식(6)
			IloLinearNumExpr[] con6 = new IloLinearNumExpr[n];
			for(int i=1; i<n; i++) {
				con6[i] = cplex.linearNumExpr();
				for(int j=0; j<n; j++) {
					con6[i].addTerm(z[j][i], 1);
					con6[i].addTerm(z[i][j], -1);
				}
				cplex.addEq(con6[i], d[i]);
			}
			
			//제약식(7-1)
			IloNumExpr[] con7 = new IloNumExpr[K];
			for(int k=0; k<K; k++) {
				con7[k] = cplex.quadNumExpr();
				for(int i=0; i<n; i++) {
					for(int j=0; j<n; j++) {
						double tmp = std[i][j][k]*std[i][j][k];
						con7[k] = cplex.sum(con7[k], cplex.prod(tmp, x[i][j][k],x[i][j][k]));
					}
				}
				con7[k] = cplex.sum(con7[k], cplex.prod(-1, w[k],w[k]));
				cplex.addLe(con7[k],0);
			}
			
			//제약식(7-2)
			IloLinearNumExpr[] con8 = new IloLinearNumExpr[K];
			IloLinearNumExpr[] con9 = new IloLinearNumExpr[K];
			for(int k=0; k<K; k++) {
				con8[k] = cplex.linearNumExpr();
				con9[k] = cplex.linearNumExpr();
				for(int i=0; i<n; i++) {
					for(int j=0; j<n; j++) {
						con8[k].addTerm(D[i][j]/v[i][j][k], x[i][j][k]);
						con9[k].addTerm(s[i], x[j][i][k]);
					}
				}
				cplex.addLe(cplex.sum(con8[k], cplex.prod(PI, w[k])), cplex.sum(B[k], cplex.prod(-1,con9[k])));
			}
			
			//제약식(8)
			for(int i=0; i<n; i++) {
				for(int k=0; k<K; k++) {
					cplex.addEq(x[i][i][k], 0);
				}
			}
			
			cplex.setParam(IloCplex.DoubleParam.TimeLimit, 3600);
			
			long start = System.currentTimeMillis();
			cplex.solve();
			double obj = cplex.getObjValue();
			long end = System.currentTimeMillis();
			double time = (end - start) / 1000.0;
			System.out.println("obj = "+obj);
			System.out.println("총 얼마? = "+cplex.getObjValue()*9500/60);
			BufferedWriter fw = new BufferedWriter(new FileWriter(filePath+fileName+fileType, true));
			
			fw.write(Double.toString(obj));
			fw.newLine();
			fw.write("K: "+K);
			fw.newLine();
			fw.write("PI: "+PI);
			fw.newLine();
			fw.write("B[k]: "+B[0]);
			fw.newLine();
			fw.write("Q: "+Q);
			fw.newLine();
			double TotalTime = 0;
			int knum = 0;
			int tmpk = -1;
			for (int k = 0; k < K; k++) {
	            int i = 0;
	            int j = 0;
	            TotalTime = 0;
	            while(j != n) {
	               if (cplex.getValue(x[i][j][k]) > 0.5) {
	            	  
	            	  TotalTime += (D[i][j]*10)/(GetData.speed[i][j]/60.*1000) + s[j];
	                  System.out.println((k+1) + "차량 이동경로 : " + (i+1) + " " + (j+1) +"\t TotalTime: "+(int)TotalTime+"분");
	                  fw.write((k+1) + "차량 이동경로 : " + (i+1) + " " + (j+1));
	                  fw.newLine();
	                  if(tmpk!=k) {
	                	  knum++;
	                	  tmpk = k;
	                  }
	                  i = j;
	                  j = -1;
	                  if(i == 0) break;
	               }
	               j++;
	            }
	            
	         }
			cplex.end();
			System.out.println(knum);
			fw.write("사용된 차량의 수 = "+knum);
			fw.newLine();
			fw.write("실행시간 = " + time + "초");
			System.out.println("실행시간 = " + time + "초");
			fw.flush();
			fw.close();
			
		}
		catch(IloException exc) {
			exc.printStackTrace();
		}
	}

}
