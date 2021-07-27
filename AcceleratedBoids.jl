using Base:Float32, aligned_sizeof, alignment
using GLMakie

GLMakie.activate!()

##

NUM_BOIDS = 70
VIEW_DISTANCE = 70 
VIEW_ANGLE = 4π/3
SIDE_MARGIN = 50

# Boid Modifiers

ALIGNMENT_MODIFIER = 0.01
COHESION_MODIFIER = 0.1
SEPERATION_MODIFIER = 0.01

MAX_VELOCITY = 120
MIN_VELOCITY = 10
MAX_ACCELERATION = 10

# Animation settings
FRAMERATE = 30
SECONDS = 30

# Plot Settings
X_WIDTH = 1000
Y_WIDTH = 1000

mutable struct Boids
    x::Vector{Float64} # m
    y::Vector{Float64} # m
    xv::Vector{Float64} # m/s
    yv::Vector{Float64} # m/s
end

Boids() = Boids(rand(NUM_BOIDS) * X_WIDTH, rand(NUM_BOIDS) * Y_WIDTH, rand(NUM_BOIDS)*(MAX_VELOCITY) .- MIN_VELOCITY, rand(NUM_BOIDS)*(MAX_VELOCITY) .- MIN_VELOCITY)

##

function iterateBoids!(boidsArr)

    # Iterate Position based on velocity
    boidsArr.x = boidsArr.x .+ (boidsArr.xv ./ FRAMERATE)
    boidsArr.y = boidsArr.y .+ (boidsArr.yv ./ FRAMERATE)


    for i in 1:NUM_BOIDS
        num_in_range = 0
        xvttl = 0
        yvttl = 0
        xps = []
        yps = []        

        dist = .√((boidsArr.x .- boidsArr.x[i]) .^ 2 .+ (boidsArr.y .- boidsArr.y[i]) .^ 2)

        for inner in 1:NUM_BOIDS
            if dist[inner] > 0.2 && dist[inner] < VIEW_DISTANCE
                num_in_range += 1
                push!(xps, boidsArr.x[inner])
                push!(yps, boidsArr.y[inner])
                xvttl = boidsArr.xv[inner]
                yvttl += boidsArr.yv[inner]
            end
        end

        if (num_in_range > 0)
            xavg = sum(xps) / num_in_range
            yavg = sum(yps) / num_in_range
            xvavg = xvttl / num_in_range
            yvavg = yvttl / num_in_range

            #ALIGNMENT
            alignmentXA = (xvavg - boidsArr.xv[i])
            alignmentYA = (yvavg - boidsArr.yv[i])

            alignmentMagnitude = √(alignmentXA^2 + alignmentYA^2)

            # alignmentXA, alignmentYA = scaleAcceleration(alignmentXA, alignmentYA)

            alignmentXA = alignmentXA * ALIGNMENT_MODIFIER
            alignmentYA = alignmentYA * ALIGNMENT_MODIFIER

            #COHESION
            targetCohesionXA = boidsArr.x[i] - xavg
            cohesionXA = ((xavg - boidsArr.x[i]) - boidsArr.xv[i])
            cohesionYA = ((yavg - boidsArr.y[i]) - boidsArr.yv[i])

            # cohesionXA, cohesionYA = scaleAcceleration(cohesionXA, cohesionYA)
            cohesionXA = cohesionXA*COHESION_MODIFIER
            cohesionYA = cohesionYA*COHESION_MODIFIER
            
            #SEPERATION

            xps = xps .- boidsArr.x[i]
            yps = yps .- boidsArr.y[i]
            xps = map(seperationInverter, xps)
            yps = map(seperationInverter, yps)

            seperationXA = (sum(xps) - boidsArr.xv[i])
            seperationYA = (sum(yps)- boidsArr.yv[i] )

            # seperationXA, seperationYA = scaleAcceleration(seperationXA, seperationYA)
            seperationXA = seperationXA * SEPERATION_MODIFIER
            seperationYA = seperationYA * SEPERATION_MODIFIER
            
            XA = alignmentXA + cohesionXA + seperationXA
            YA = alignmentYA + cohesionYA + seperationYA
        else
            XA = 0
            YA = 0
        end

        if getMagnitude(XA, YA) > MAX_ACCELERATION
            print("X ", XA)
            print(" Y ", YA)
            print(" sepX ", seperationXA)
            print(" CohX ", cohesionXA)
            print(" AlgX ", alignmentXA)
            XA, YA = scaleAcceleration(XA, YA).*MAX_ACCELERATION
            println(" XAdj ", XA, " YAdj ", YA)
        end

        boidsArr.xv[i] += (XA / FRAMERATE)
        boidsArr.yv[i] += (YA / FRAMERATE)
    end
    # limit velocities
    # boidsArr.xv = map(gateVelocity, boidsArr.xv)
    # boidsArr.yv = map(gateVelocity, boidsArr.yv)
    # for i in 1:NUM_BOIDS
    #     if boidsArr.xv[i] > MAX_VELOCITY
    #         boidsArr.xv[i] = MAX_VELOCITY
    #     elseif boidsArr.xv[i] < MIN_VELOCITY
    #         boidsArr.xv[i] = MIN_VELOCITY
    #     end
    #     if boidsArr.yv[i] > MAX_VELOCITY
    #         boidsArr.yv[i] = MAX_VELOCITY
    #     elseif boidsArr.yv[i] < MIN_VELOCITY
    #         boidsArr.yv[i] = MIN_VELOCITY
    #     end
    # end
    for i in 1:NUM_BOIDS
        #Scale velocity to max / min
        if getMagnitude(boidsArr.xv[i], boidsArr.yv[i]) > MAX_VELOCITY
            newxv, newyv = scaleAcceleration(boidsArr.xv[i], boidsArr.yv[i])
            boidsArr.xv[i] = newxv*MAX_VELOCITY
            boidsArr.yv[i] = newyv*MAX_VELOCITY
        elseif getMagnitude(boidsArr.xv[i], boidsArr.yv[i]) < MIN_VELOCITY
            newxv, newyv = scaleAcceleration(boidsArr.xv[i], boidsArr.yv[i])
            boidsArr.xv[i] = newxv*MIN_VELOCITY
            boidsArr.yv[i] = newyv*MIN_VELOCITY
        end

        #Prevent from escaping (keep within bounds)
        if boidsArr.x[i] > X_WIDTH - SIDE_MARGIN
            boidsArr.xv[i] -= 1
        elseif boidsArr.x[i] < SIDE_MARGIN
            boidsArr.xv[i] += 1
        end
        if boidsArr.y[i] > Y_WIDTH - SIDE_MARGIN
            boidsArr.yv[i] -= 1
        elseif boidsArr.y[i] < SIDE_MARGIN
            boidsArr.yv[i] += 1
        end
    end
end

#Poorly written helper functions

function seperationInverter(val)
    if val < 0
        return -VIEW_DISTANCE - val
    else 
        return VIEW_DISTANCE - val
    end
end

function scaleAcceleration(xa, ya)
    magnitude = √(xa^2 + ya^2)
    xa = xa / magnitude
    ya = ya / magnitude
    return (xa, ya)
end

function getMagnitude(xa, ya)
    return √(xa^2 + ya^2)
end
##
# Define Figure

fig = Figure(resolution=(X_WIDTH, Y_WIDTH))

ax = fig[1, 1] = Axis(fig)

hidespines!(ax)
hidedecorations!(ax)

limits!(ax, 0, X_WIDTH, 0, Y_WIDTH)

##
boids = Boids()

timestamps = range(0, SECONDS, step=1 / FRAMERATE)

record(fig, "randimation.mp4", timestamps; framerate = FRAMERATE) do i
    empty!(ax)
    iterateBoids!(boids)
    # tans = atan.(boids.yv ./ boids.xv)
    scatter!(ax, boids.x, boids.y, color = :black)
end

##
 