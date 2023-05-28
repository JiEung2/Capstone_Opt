package Cap;
import java.io.*;
import java.nio.file.Paths;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.*;

public class GetData {

	public static int n = 0;
	public static int K = 0;
	public static int Q = 0;
	public static double[][] d;
	public static int[][] speed;
	public static int[] delivery;
	public static int[] pickup;
	public static double[][] std;
	
	public static void main(String[] args) {
		
		Path path = Paths.get("/Users/hijieung/Desktop/Opt_Lab/Data/P-n19-k2_Instance.txt");
		
		
		try {
			List<String> allLines = Files.readAllLines(path);
			n= Integer.parseInt(allLines.get(0));
			K= Integer.parseInt(allLines.get(1));
			Q= Integer.parseInt(allLines.get(2));
			d = new double [n][n];
			speed = new int[n][n];
			delivery = new int[n+1];
			pickup = new int[n+1];
			std = new double[n][n];
			
			for(int i=0; i<n; i++) {
				String[] tmpdata = allLines.get(i+4).split(" ");
				for(int j=0; j<n; j++) {
					d[i][j] = Double.parseDouble(tmpdata[j]);
				}
			}

			for(int i=0; i<n; i++) {
				String[] tmpdata = allLines.get(i+n+5).split(" ");
				for(int j=0; j<n; j++) {
					speed[i][j] = Integer.parseInt(tmpdata[j]);
				}
			}
			
			for(int i=0; i<n; i++) {
				String[] tmpdata = allLines.get(i+(2*n)+6).split(" ");
				for(int j=0; j<n; j++) {
					std[i][j] = Double.parseDouble(tmpdata[j]);
				}
			}
			
			String[] dtmp = allLines.get(7+n*3).split(" ");
			for(int i=0; i<n; i++) {
				delivery[i+1] = Integer.parseInt(dtmp[i]);
			}

			String[] ptmp = allLines.get(8+n*3).split(" ");
			for(int i=0; i<n; i++) {
				pickup[i+1] = Integer.parseInt(ptmp[i]);
			}
			
			
			System.out.println("n: "+n);
			System.out.println("K: "+K);
			System.out.println("Q: "+Q);
			System.out.println();
			System.out.println("거리는");
			for(int i=0; i<n; i++) {
				for(int j=0; j<n; j++) {
					System.out.print((int)d[i][j]+" ");
				}
				System.out.println();
			}
			System.out.println();
			System.out.println("속력은");
			for(int i=0; i<n; i++) {
				for(int j=0; j<n; j++) {
					System.out.print(speed[i][j]+" ");
				}
				System.out.println();
			}
			System.out.println();
			System.out.println("표준편차");
			for(int i=0; i<n; i++) {
				for(int j=0; j<n; j++) {
					System.out.print(std[i][j]+" ");
				}
				System.out.println();
			}
			int sum=0;
			System.out.println();
			for(int i=1; i<=n; i++) {
				System.out.print(delivery[i]+" ");
				sum += delivery[i];
			}
			System.out.println(sum);
			sum=0;
			for(int i=1; i<=n; i++) {
				System.out.print(pickup[i]+" ");
				sum+=pickup[i];
			}
			System.out.println(sum);
		}
		catch(IOException ie) {
			ie.printStackTrace();
		}
		
		
	}

}
