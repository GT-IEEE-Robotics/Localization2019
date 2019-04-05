<<<<<<< HEAD
import math
import cmath
import random
import matplotlib.pyplot as plt

# generates random points forming a square
# first point is generated to be in [0, 100)
# rest of the points may lie outside of [0, 100)
# side_len is the side length
# delta describes the acceptable distance from the rectangle
def random_square(side_len, delta):
    
    #generates corners: c1, c2, c3, c4
    c1 = random.randrange(100) + 1j*random.randrange(100)
    diag_len = side_len*math.sqrt(2)
    c2 = c1 + diag_len*cmath.exp(1j*random.uniform(-1*math.pi, math.pi))
    midpt = c1 + (c2-c1).real/2 + 1j*(c2-c1).imag/2
    c3 = midpt + (midpt-c1)*cmath.exp(1j*math.pi/2)
    c4 = midpt + (midpt-c1)*cmath.exp(-1j*math.pi/2)
    
    #draws sides
    s1, s2 = random_error(draw_line(c1, c3), delta), random_error(draw_line(c1, c4), delta)
    s3, s4 = random_error(draw_line(c2, c3), delta), random_error(draw_line(c2, c4), delta)

    #converts to real parts
    square = real_tuples(s1+s2+s3+s4)
    return [(point[0], point[1], 'lidar') for point in square]


# much like random_square(), but a rectangle
def random_rectangle(side_len_1, side_len_2, delta):

    #generates corners: c1, c2, c3, c4
    c1 = random.randrange(100) + 1j*random.randrange(100)
    diag_len = math.hypot(side_len_1, side_len_2)
    c2 = c1 + diag_len*cmath.exp(1j*random.uniform(-1*math.pi, math.pi))
    midpt = c1 + (c2-c1).real/2 + 1j*(c2-c1).imag/2
    c3 = midpt + (midpt-c1)*cmath.exp(1j*math.acos(1-(side_len_1**2)/(2*(diag_len/2)**2)))
    c4 = midpt + (midpt-c2)*cmath.exp(1j*math.acos(1-(side_len_1**2)/(2*(diag_len/2)**2))) 

    #draws sides
    s1, s2 = random_error(draw_line(c1, c3), delta), random_error(draw_line(c1, c4), delta)
    s3, s4 = random_error(draw_line(c2, c3), delta), random_error(draw_line(c2, c4), delta)

    #converts to real parts
    return real_tuples(s1+s2+s3+s4)


# draws a random polygon of n sides
# size is roughly the radius
# can be either regular or irregular
def random_ngon(n_sides, size, delta, regular='False'):
    center = random.randrange(25, 75) + 1j*random.randrange(25, 75)
    if regular==True:
        verts = [center + size*cmath.exp(2j*math.pi*n/n_sides) for n in range(n_sides)]
    else:
        verts = [center + size*random.uniform(0.5, 1.5)*cmath.exp(2j*math.pi*n*random.uniform(0.8, 1.2)/n_sides) for n in range(n_sides)]
    ngon = random_error(draw_line(verts[0], verts[len(verts)-1]), delta)
    for n in range(len(verts)-1):
        ngon += random_error(draw_line(verts[n], verts[n+1]), delta)
    return real_tuples(ngon)


# for when you just don't care
def random_shape():
    x = random.randint(0, 3)
    if x == 0:
        return(random_square(random.randint(30, 70), random.uniform(0, 2)))
    elif x == 1:
        return(random_rectangle(random.randint(30, 70), random.randint(30, 70), random.uniform(0, 2)))
    elif x == 2:
        return(random_ngon(random.randint(3, 8), random.randint(20, 40), random.uniform(0, 2), regular='True'))
    else:
        return(random_ngon(random.randint(3, 8), random.randint(20, 40), random.uniform(0, 2)))


# returns a list of complex numbers corresponding to a line
def draw_line(point1, point2):
    return [point1 + r*cmath.exp(1j*cmath.phase(point2-point1)) for r in range(int(abs(point2-point1)))]

# adds randomness to points
def random_error(points, delta):
    p1, p2 = points[0], points[len(points)-1]
    return [p+random.uniform(-1*delta, delta)*cmath.exp(1j*(cmath.phase(p2-p1)+math.pi/2)) for p in points]

# converts complex numbers to real tuples
def real_tuples(nums):
    return [(n.real, n.imag) for n in nums]

################################
# everything below this line is for testing purposes
def tuple_plt(nums, spec=''):
    plt.plot([n[0] for n in nums], [n[1] for n in nums], spec)

# class __main__:
#     tuple_plt(random_square(), '.')
#     plt.xlim([-50, 150])
#     plt.ylim([-50, 150])
#     plt.show()

# tuple_plt(random_square(100, 5), '.')
# plt.show()
=======
import math
import cmath
import random
import matplotlib.pyplot as plt

# generates random points forming a square
# first point is generated to be in [0, 100)
# rest of the points may lie outside of [0, 100)
# side_len is the side length
# delta describes the acceptable distance from the rectangle
def random_square(side_len, delta):
    
    #generates corners: c1, c2, c3, c4
    c1 = random.randrange(100) + 1j*random.randrange(100)
    diag_len = side_len*math.sqrt(2)
    c2 = c1 + diag_len*cmath.exp(1j*random.uniform(-1*math.pi, math.pi))
    midpt = c1 + (c2-c1).real/2 + 1j*(c2-c1).imag/2
    c3 = midpt + (midpt-c1)*cmath.exp(1j*math.pi/2)
    c4 = midpt + (midpt-c1)*cmath.exp(-1j*math.pi/2)
    
    #draws sides
    s1, s2 = random_error(draw_line(c1, c3), delta), random_error(draw_line(c1, c4), delta)
    s3, s4 = random_error(draw_line(c2, c3), delta), random_error(draw_line(c2, c4), delta)

    #converts to real parts
    square = real_tuples(s1+s2+s3+s4)
    square = [(position[0], position[1], 'lidar') for position in square]


# much like random_square(), but a rectangle
def random_rectangle(side_len_1, side_len_2, delta):

    #generates corners: c1, c2, c3, c4
    c1 = random.randrange(100) + 1j*random.randrange(100)
    diag_len = math.hypot(side_len_1, side_len_2)
    c2 = c1 + diag_len*cmath.exp(1j*random.uniform(-1*math.pi, math.pi))
    midpt = c1 + (c2-c1).real/2 + 1j*(c2-c1).imag/2
    c3 = midpt + (midpt-c1)*cmath.exp(1j*math.acos(1-(side_len_1**2)/(2*(diag_len/2)**2)))
    c4 = midpt + (midpt-c2)*cmath.exp(1j*math.acos(1-(side_len_1**2)/(2*(diag_len/2)**2))) 

    #draws sides
    s1, s2 = random_error(draw_line(c1, c3), delta), random_error(draw_line(c1, c4), delta)
    s3, s4 = random_error(draw_line(c2, c3), delta), random_error(draw_line(c2, c4), delta)

    #converts to real parts
    return real_tuples(s1+s2+s3+s4)


# draws a random polygon of n sides
# size is roughly the radius
# can be either regular or irregular
def random_ngon(n_sides, size, delta, regular='False'):
    center = random.randrange(25, 75) + 1j*random.randrange(25, 75)
    if regular==True:
        verts = [center + size*cmath.exp(2j*math.pi*n/n_sides) for n in range(n_sides)]
    else:
        verts = [center + size*random.uniform(0.5, 1.5)*cmath.exp(2j*math.pi*n*random.uniform(0.8, 1.2)/n_sides) for n in range(n_sides)]
    ngon = random_error(draw_line(verts[0], verts[len(verts)-1]), delta)
    for n in range(len(verts)-1):
        ngon += random_error(draw_line(verts[n], verts[n+1]), delta)
    return real_tuples(ngon)


# for when you just don't care
def random_shape():
    x = random.randint(0, 3)
    if x == 0:
        return(random_square(random.randint(30, 70), random.uniform(0, 2)))
    elif x == 1:
        return(random_rectangle(random.randint(30, 70), random.randint(30, 70), random.uniform(0, 2)))
    elif x == 2:
        return(random_ngon(random.randint(3, 8), random.randint(20, 40), random.uniform(0, 2), regular='True'))
    else:
        return(random_ngon(random.randint(3, 8), random.randint(20, 40), random.uniform(0, 2)))


# returns a list of complex numbers corresponding to a line
def draw_line(point1, point2):
    return [point1 + r*cmath.exp(1j*cmath.phase(point2-point1)) for r in range(int(abs(point2-point1)))]

# adds randomness to points
def random_error(points, delta):
    p1, p2 = points[0], points[len(points)-1]
    return [p+random.uniform(-1*delta, delta)*cmath.exp(1j*(cmath.phase(p2-p1)+math.pi/2)) for p in points]

# converts complex numbers to real tuples
def real_tuples(nums):
    return [(n.real, n.imag) for n in nums]

################################
# everything below this line is for testing purposes
def tuple_plt(nums, spec=''):
    plt.plot([n[0] for n in nums], [n[1] for n in nums], spec)

class __main__:
    tuple_plt(random_shape(), '.')
    plt.xlim([-50, 150])
    plt.ylim([-50, 150])
    plt.show()
>>>>>>> 620518bd9d022f5f9b34bf0cd77cf16b5ea8c03b
