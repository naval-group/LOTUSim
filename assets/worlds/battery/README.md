

### Battery plugin experimentation 


Make LRAUV move 

```bash 
gz topic -t /model/lrauv/joint/propeller_joint/cmd_thrust -m gz.msgs.Double -p 'data: 15'
```

Check new topic "power"

```bash 
gz topic -e -t /model/lrauv/battery/power
```




## Contributing

### Workflow
We use the Gitflow collaborating workflow. You can find the explanation of this workflow [here](https://www.atlassian.com/git/tutorials/comparing-workflows/gitflow-workflow).

We are using the default Gitflow branch naming like [here](https://www.gitkraken.com/blog/gitflow).

### Issues

When you open an issue, you need to put the correct label corresponding to the category of the issue e.g. ~bug or ~suggestion. It needs to be written in english.

**IMPORTANT** You have to put ~"project::development" or ~"project::management" label on each issue as they are used for filtering in the different issue boards

You can find the description of the labels [here](https://developers.naval-group.com/gitlab/naval-group/naval-group-pacific/lotusim/-/labels).


