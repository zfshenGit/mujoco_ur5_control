import gym

env = gym.make('car2D:car2D-v0')
env.reset()

for i in range(10000):
    # action = [0.2, 0.2, 0.2, 0.2] # sample random action
    action = env.action_space.sample()
    print(action)
    obs, reward, done, info = env.step(action)  # take action in the environment
    env.render()  # render on display
