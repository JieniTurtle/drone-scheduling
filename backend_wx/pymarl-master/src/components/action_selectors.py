import torch as th
from torch.distributions import Categorical
from .epsilon_schedules import DecayThenFlatSchedule

REGISTRY = {}


class MultinomialActionSelector():

    def __init__(self, args):
        self.args = args

        self.schedule = DecayThenFlatSchedule(args.epsilon_start, args.epsilon_finish, args.epsilon_anneal_time,
                                              decay="linear")
        self.epsilon = self.schedule.eval(0)
        self.test_greedy = getattr(args, "test_greedy", True)

    def select_action(self, agent_inputs, avail_actions, t_env, test_mode=False):
        masked_policies = agent_inputs.clone()
        masked_policies[avail_actions == 0.0] = 0.0

        self.epsilon = self.schedule.eval(t_env)

        if test_mode and self.test_greedy:
            picked_actions = masked_policies.max(dim=2)[1]
        else:
            picked_actions = Categorical(masked_policies).sample().long()

        return picked_actions


REGISTRY["multinomial"] = MultinomialActionSelector


class EpsilonGreedyActionSelector():

    def __init__(self, args):
        self.args = args

        self.schedule = DecayThenFlatSchedule(args.epsilon_start, args.epsilon_finish, args.epsilon_anneal_time,
                                              decay="linear")
        self.epsilon = self.schedule.eval(0)

    def select_action(self, agent_inputs, avail_actions, t_env, test_mode=False):

        # Assuming agent_inputs is a batch of Q-Values for each agent bav
        self.epsilon = self.schedule.eval(t_env)

        if test_mode:
            # Greedy action selection only
            self.epsilon = 0.0

        # mask actions that are excluded from selection
        masked_q_values = agent_inputs.clone()
        masked_q_values[avail_actions == 0.0] = -float("inf")  # should never be selected!

        random_numbers = th.rand_like(agent_inputs[:, :, 0])
        pick_random = (random_numbers < self.epsilon).long()
        random_actions = Categorical(avail_actions.float()).sample().long()

        if getattr(self.args, "unique_task_assignment", False):
            greedy_actions = self._select_unique_task_actions(masked_q_values, avail_actions)
        else:
            greedy_actions = masked_q_values.max(dim=2)[1]

        picked_actions = pick_random * random_actions + (1 - pick_random) * greedy_actions
        return picked_actions

    def _select_unique_task_actions(self, masked_q_values, avail_actions):
        """Choose joint actions with max summed Q while avoiding duplicate non-NoOp tasks."""
        bs, n_agents, _ = masked_q_values.shape
        noop_action = int(getattr(self.args, "noop_action", 0))
        out = th.zeros((bs, n_agents), dtype=th.long, device=masked_q_values.device)

        for b in range(bs):
            candidates = []
            for agent_idx in range(n_agents):
                valid = th.nonzero(avail_actions[b, agent_idx] > 0, as_tuple=False).squeeze(-1)
                if valid.numel() == 0:
                    valid = th.tensor([noop_action], dtype=th.long, device=masked_q_values.device)
                qvals = masked_q_values[b, agent_idx, valid]
                order = th.argsort(qvals, descending=True)
                candidates.append(valid[order].tolist())

            best_score = -float("inf")
            best_actions = [noop_action for _ in range(n_agents)]
            current = [noop_action for _ in range(n_agents)]

            def dfs(agent_idx, used_tasks, score):
                nonlocal best_score, best_actions
                if agent_idx == n_agents:
                    if score > best_score:
                        best_score = score
                        best_actions = current.copy()
                    return

                for action in candidates[agent_idx]:
                    if action != noop_action and action in used_tasks:
                        continue

                    current[agent_idx] = action
                    next_used = used_tasks
                    if action != noop_action:
                        next_used = used_tasks | {action}

                    q = float(masked_q_values[b, agent_idx, action].item())
                    dfs(agent_idx + 1, next_used, score + q)

            dfs(0, set(), 0.0)
            out[b] = th.tensor(best_actions, dtype=th.long, device=masked_q_values.device)

        return out


REGISTRY["epsilon_greedy"] = EpsilonGreedyActionSelector
