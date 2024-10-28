## Cubic Spline

For a group of certain points $(x_{0,1},x_{0,2},\cdots,x_{0,m}),(x_{1,1},x_{1,2},\cdots,x_{1,m}), (x_{2,1},x_{2,2},\cdots,x_{2,m}), \cdots, (x_{n,1},x_{n,2},\cdots,x_{n,m})$, we want to find a smooth enough curve that passes all these points to achieve the interpolation goal. To do so, we can use piecewise cubic functions to fitting the points.

Let's talk about a specific dimention $k$ first. To simplify the denotation, we let $x_i=x_{i,k}$. For a set of $n+1$ points $(x_{0,k},x_{1,k},x_{2,k},\cdots,x_{n,k})$ and for the $i$-th piece of the spline, we can represent it by

$$
S_i(t)=a_it^3+b_it^2+c_it+d_i, \ \ \ \ t\in[0,1],\ i=0,1,\cdots,n-1
$$

To ensure the smooth of the curve, we write down the constraints,

$$
\begin{cases}
S_{i-1}(1)=S_i(0)=x_{i} \\
S_{i-1}'(1)=S_i'(0) \\
S_{i-1}''(1)=S_i''(0)
\end{cases}
$$

Apply $a_i,b_i,c_i,d_i$ to the constraints, we get

$$
\begin{cases}
a_{i-1}+b_{i-1}+c_{i-1}+d_{i-1}=d_i \\
3a_{i-1}+2b_{i-1}+c_{i-1}=c_i \\
6a_{i-1}+2b_{i-1}=2b_i \\
\end{cases}
$$

So we can get the following expressions

$$
\begin{cases}
a_i=x_{i+2}-2x_{i+1}+x_{i} \\
b_i=-x_{i+2}+2x_{i+1,k}-x_{i} \\
c_{i}=x_{i+1}-x_{i} \\
d_i=x_{i}
\end{cases}\ ,\ i=1,2,\cdots,n-1
$$

### Open Loop

Assume stationary boundary $S_0'(0)=0,S_{n-1}'(1)=0$, which is $c_0=0, 3a_{n-1}+2b_{n-1}+c_{n-1}=0$. 

You can suppose $x_{n+2}=x_{n+1}$, so that $x_{n+2}-2x_{n+1}+x_n=-x_{n+1}+x_n$

Convert the expressions into matrix form, we have

$$
\left[\begin{matrix}
a_0\\a_1\\a_2\\a_3\\\vdots\\a_{n-2}\\a_{n-1}\\
\end{matrix}\right]=
\left[\begin{matrix}
2&-3&1&0&\cdots&0&0&0\\
0&1&-2&1&\cdots&0&0&0\\
0&0&1&-2&\cdots&0&0&0\\
0&0&0&1&\cdots&0&0&0\\
\vdots&\vdots&\vdots&\vdots&\ddots&\vdots&\vdots&\vdots\\
0&0&0&0&\cdots&1&-2&1\\
0&0&0&0&\cdots&0&1&-1\\
\end{matrix}\right]
\left[\begin{matrix}
x_0\\x_1\\x_2\\x_3\\\vdots\\x_{n-2}\\x_{n-1}\\x_{n}
\end{matrix}\right]
$$

$$
\left[\begin{matrix}
b_0\\b_1\\b_2\\b_3\\\vdots\\b_{n-2}\\b_{n-1}\\
\end{matrix}\right]=
\left[\begin{matrix}
-3&4&-1&0&\cdots&0&0&0\\
0&-1&2&-1&\cdots&0&0&0\\
0&0&-1&2&\cdots&0&0&0\\
0&0&0&-1&\cdots&0&0&0\\
\vdots&\vdots&\vdots&\vdots&\ddots&\vdots&\vdots&\vdots\\
0&0&0&0&\cdots&-1&2&-1\\
0&0&0&0&\cdots&0&-1&1\\
\end{matrix}\right]
\left[\begin{matrix}
x_0\\x_1\\x_2\\x_3\\\vdots\\x_{n-2}\\x_{n-1}\\x_{n}
\end{matrix}\right]
$$

$$
\left[\begin{matrix}
c_0\\c_1\\c_2\\c_3\\\vdots\\c_{n-2}\\c_{n-1}\\
\end{matrix}\right]=
\left[\begin{matrix}
0&0&0&0&\cdots&0&0&0\\
0&-1&1&0&\cdots&0&0&0\\
0&0&-1&1&\cdots&0&0&0\\
0&0&0&-1&\cdots&0&0&0\\
\vdots&\vdots&\vdots&\vdots&\ddots&\vdots&\vdots&\vdots\\
0&0&0&0&\cdots&-1&1&0\\
0&0&0&0&\cdots&0&-1&1\\
\end{matrix}\right]
\left[\begin{matrix}
x_0\\x_1\\x_2\\x_3\\\vdots\\x_{n-2}\\x_{n-1}\\x_{n}
\end{matrix}\right]
$$

$$
\left[\begin{matrix}
d_0\\d_1\\d_2\\d_3\\\vdots\\d_{n-2}\\d_{n-1}\\
\end{matrix}\right]=
\left[\begin{matrix}
1&0&0&0&\cdots&0&0&0\\
0&1&0&0&\cdots&0&0&0\\
0&0&1&0&\cdots&0&0&0\\
0&0&0&1&\cdots&0&0&0\\
\vdots&\vdots&\vdots&\vdots&\ddots&\vdots&\vdots&\vdots\\
0&0&0&0&\cdots&1&0&0\\
0&0&0&0&\cdots&0&1&0\\
\end{matrix}\right]
\left[\begin{matrix}
x_0\\x_1\\x_2\\x_3\\\vdots\\x_{n-2}\\x_{n-1}\\x_{n}
\end{matrix}\right]
$$

### Closed Loop

If the curve is a closed loop, $x_0=x_{n+1}$. Then $S_0'(0)=S_{n-1}'(1), S_0''(0)=S_{n-1}''(1)$, which means $c_0=3a_{n-1}+2b_{n-1}+c_{n-1}$ and $3a_{n-1}+b_{n-1}=b_0$.

$$
\left[\begin{matrix}
a_0\\a_1\\a_2\\a_3\\\vdots\\a_{n-1}\\a_n
\end{matrix}\right]=
\left[\begin{matrix}
1&-2&1&0&\cdots&0&0\\
0&1&-2&1&\cdots&0&0\\
0&0&1&-2&\cdots&0&0\\
0&0&0&1&\cdots&0&0\\
\vdots&\vdots&\vdots&\vdots&\ddots&\vdots&\vdots\\
1&0&0&0&\cdots&1&-2\\
-2&1&0&0&\cdots&0&1\\
\end{matrix}\right]
\left[\begin{matrix}
x_0\\x_1\\x_2\\x_3\\\vdots\\x_{n-1}\\x_n
\end{matrix}\right]
$$

$$
\bf{b=-a}
$$

$$
\left[\begin{matrix}
c_0\\c_1\\c_2\\c_3\\\vdots\\c_{n-1}\\c_n
\end{matrix}\right]=
\left[\begin{matrix}
-1&1&0&0&\cdots&0&0\\
0&-1&1&0&\cdots&0&0\\
0&0&-1&1&\cdots&0&0\\
0&0&0&-1&\cdots&0&0\\
\vdots&\vdots&\vdots&\vdots&\ddots&\vdots&\vdots\\
0&0&0&0&\cdots&-1&1\\
1&0&0&0&\cdots&0&-1\\
\end{matrix}\right]
\left[\begin{matrix}
x_0\\x_1\\x_2\\x_3\\\vdots\\x_{n-1}\\x_n
\end{matrix}\right]
$$

$$
\bf{d=x_k}
$$
